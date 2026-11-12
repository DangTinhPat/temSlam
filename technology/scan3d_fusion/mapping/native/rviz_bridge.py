"""Publish the native mapper's saved map files for RViz; does not touch SLAM or control."""
import argparse
import json
import math
import struct
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "gui/rb1_gui"))
from cad_viewer import RbModel


def read_pgm(path):
    data = path.read_bytes()
    tokens = []
    cursor = 0
    while len(tokens) < 4:
        while cursor < len(data) and data[cursor] in b" \t\r\n":
            cursor += 1
        if data[cursor:cursor + 1] == b"#":
            cursor = data.index(b"\n", cursor) + 1
            continue
        start = cursor
        while cursor < len(data) and data[cursor] not in b" \t\r\n":
            cursor += 1
        tokens.append(data[start:cursor])
    magic, width, height, maximum = tokens
    if magic != b"P5" or int(maximum) != 255:
        raise ValueError(f"unsupported PGM format in {path}")
    if data[cursor:cursor + 2] == b"\r\n":
        cursor += 2
    else:
        cursor += 1
    return int(width), int(height), data[cursor:cursor + int(width) * int(height)]


def read_cloud(path):
    with path.open() as source:
        for line in source:
            if line.strip() == "end_header":
                break
        else:
            raise ValueError(f"missing PLY header in {path}")
        return [tuple(map(float, line.split()[:3])) for line in source if line.strip()]


class MapBridge(Node):
    def __init__(self, session, scene):
        super().__init__("native_tof_rviz_bridge")
        self.session = session
        self.scene_path = scene
        self.scene_markers = None
        self.map_origin = None
        self.has_valid_pose = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latched)
        self.cloud_pub = self.create_publisher(PointCloud2, "/tof/occupied_voxels", latched)
        self.scene_pub = self.create_publisher(MarkerArray, "/environment", latched)
        self.robot_pub = self.create_publisher(MarkerArray, "/robot", latched)
        self.robot_model = read_robot(scene)
        self.create_timer(1.0, self.publish_scene)
        self.last_frame = None
        self.timer = self.create_timer(1 / 30, self.publish_latest)
        self.get_logger().info(f"Reading native map session: {session}; RViz scene: {scene}")

    def publish_latest(self):
        try:
            state = json.loads((self.session / "state.json").read_text())
            stamp = self.get_clock().now().to_msg()
            try:
                view = json.loads((self.session / "robot_state.json").read_text())
            except OSError:
                view = {"x": state["x"], "y": state["y"], "qw": math.cos(state["yaw"] / 2),
                        "qx": 0.0, "qy": 0.0, "qz": math.sin(state["yaw"] / 2)}
            if state["valid"]:
                self.has_valid_pose = True
            elif not self.has_valid_pose:
                try:
                    self.has_valid_pose = any(
                        line.rstrip().endswith(",1")
                        for line in (self.session / "trajectory.csv").read_text().splitlines()[1:])
                except OSError:
                    pass
            if self.map_origin is None and self.has_valid_pose:
                self.map_origin = map_origin(view, state)
                self.scene_markers = align_scene(
                    read_scene(self.scene_path, stamp), self.map_origin)
                self.publish_scene()
                self.get_logger().info("Aligned MuJoCo scene to the ToF odometry frame")
            if self.map_origin is None:
                return
            view = align_robot(view, state)
            self.robot_model.apply(view)
            robot = read_robot_markers(self.robot_model, stamp)
            self.robot_pub.publish(robot)
            if state["frame"] == self.last_frame:
                return
            width, height, pixels = read_pgm(self.session / "map.pgm")
            if len(pixels) != width * height:
                return  # Ignore files while the mapper is replacing a snapshot.
            cloud = read_cloud(self.session / "map.ply")
        except (OSError, ValueError, KeyError, IndexError):
            return

        grid = OccupancyGrid()
        grid.header = Header(stamp=stamp, frame_id="map")
        grid.info.resolution = float(state["resolution"])
        grid.info.width = width
        grid.info.height = height
        grid.info.origin = Pose()
        grid.info.origin.position.x, grid.info.origin.position.y = state["origin"]
        grid.info.origin.orientation.w = 1.0
        values = {20: 100, 245: 0, 127: -1}
        grid.data = [values.get(pixels[(height - y - 1) * width + x], -1)
                     for y in range(height) for x in range(width)]

        header = Header(stamp=stamp, frame_id="map")
        fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                  for name, offset in (("x", 0), ("y", 4), ("z", 8))]
        cloud_msg = PointCloud2(header=header, height=1, width=len(cloud), fields=fields,
                                is_bigendian=False, point_step=12, row_step=12 * len(cloud),
                                data=b"".join(struct.pack("<fff", *point) for point in cloud),
                                is_dense=True)
        self.map_pub.publish(grid)
        self.cloud_pub.publish(cloud_msg)
        self.last_frame = state["frame"]
        if self.last_frame % 30 == 0:
            self.get_logger().info(f"Published frame {self.last_frame}: {width}x{height}, {len(cloud)} voxels, {len(robot.markers)} CAD parts")

    def publish_scene(self):
        if self.scene_markers is not None:
            self.scene_pub.publish(self.scene_markers)


def yaw_from_view(view):
    return math.atan2(2 * (view["qw"] * view["qz"] + view["qx"] * view["qy"]),
                      1 - 2 * (view["qy"] ** 2 + view["qz"] ** 2))


def map_origin(view, state):
    yaw = yaw_from_view(view) - state["yaw"]
    c, s = math.cos(yaw), math.sin(yaw)
    return (view["x"] - c * state["x"] + s * state["y"],
            view["y"] - s * state["x"] - c * state["y"], yaw)


def align_robot(view, state):
    result = dict(view)
    result["x"], result["y"] = state["x"], state["y"]
    yaw_delta = state["yaw"] - yaw_from_view(view)
    c, s = math.cos(yaw_delta / 2), math.sin(yaw_delta / 2)
    w, x, y, z = (view[k] for k in ("qw", "qx", "qy", "qz"))
    result["qw"], result["qx"], result["qy"], result["qz"] = (
        c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)
    return result


def align_scene(markers, origin):
    ox, oy, yaw = origin
    c, s = math.cos(yaw), math.sin(yaw)
    half_c, half_s = math.cos(yaw / 2), math.sin(yaw / 2)
    for marker in markers.markers:
        p = marker.pose.position
        dx, dy = p.x - ox, p.y - oy
        p.x, p.y = c * dx + s * dy, -s * dx + c * dy
        q = marker.pose.orientation
        q.w, q.x, q.y, q.z = (half_c * q.w + half_s * q.z,
                              half_c * q.x + half_s * q.y,
                              half_c * q.y - half_s * q.x,
                              half_c * q.z - half_s * q.w)
    return markers


def read_scene(path, stamp):
    root = ET.parse(path).getroot()
    world = root.find("worldbody")
    meshes = {mesh.get("name"): mesh for mesh in root.find("asset").findall("mesh")}
    mesh_files = {name: mesh.get("file") for name, mesh in meshes.items()}
    markers = MarkerArray()
    ground = next((geom for geom in world.findall("geom")
                   if geom.get("name") == "ground" and geom.get("type") == "plane"), None)
    if ground is not None:
        half_size = [float(value) for value in ground.get("size", "0 0").split()]
        position = [float(value) for value in ground.get("pos", "0 0 0").split()]
        floor = Marker()
        floor.header = Header(stamp=stamp, frame_id="map")
        floor.ns, floor.id = "tiny_house_floor", 0
        floor.type, floor.action = Marker.MESH_RESOURCE, Marker.ADD
        floor.mesh_resource = (path.parent / "indoor_tiny_house_floor.obj").resolve().as_uri()
        floor.mesh_use_embedded_materials = True
        floor.pose.position.x, floor.pose.position.y, floor.pose.position.z = position[0], position[1], position[2] + .002
        floor.pose.orientation.w = 1.0
        floor.scale.x, floor.scale.y, floor.scale.z = 2*half_size[0], 2*half_size[1], 1.0
        floor.color.r = floor.color.g = floor.color.b = floor.color.a = 1.0
        markers.markers.append(floor)
    for geom in world.findall("geom"):
        name = geom.get("name", "")
        kind = geom.get("type")
        if not name.startswith(("south_wall_", "north_wall", "west_wall", "east_wall",
                                "partition_", "env_stair_tread_", "env_stair_riser_",
                                "env_stair_post_", "env_stair_rail_", "env_stair_stringer_",
                                "env_stair_landing", "env_ball_",
                                "env_cabinet", "env_round_pillar", "env_mesh_")):
            continue
        values = [float(value) for value in geom.get("size", "").split()]
        pos = [float(value) for value in geom.get("pos", "0 0 0").split()]
        rgba = [float(value) for value in geom.get("rgba", "0.7 0.7 0.7 1").split()]
        if kind == "box" and len(values) == 3:
            marker_type, scale = Marker.CUBE, [2 * value for value in values]
        elif kind == "cylinder" and len(values) == 2:
            marker_type, scale = Marker.CYLINDER, [2 * values[0], 2 * values[0], 2 * values[1]]
        elif kind == "sphere" and len(values) == 1:
            marker_type, scale = Marker.SPHERE, [2 * values[0]] * 3
        elif kind == "mesh" and geom.get("mesh") in mesh_files:
            marker_type = Marker.MESH_RESOURCE
            scale = [float(value) for value in meshes[geom.get("mesh")].get("scale", "1 1 1").split()]
        else:
            continue
        quat = [float(value) for value in geom.get("quat", "1 0 0 0").split()]
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id="map")
        marker.ns = "tiny_house"
        marker.id = len(markers.markers)
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos
        marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z = quat
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        if marker_type == Marker.MESH_RESOURCE:
            marker.mesh_resource = (path.parent / mesh_files[geom.get("mesh")]).resolve().as_uri()
        markers.markers.append(marker)
    if not markers.markers:
        raise ValueError(f"no tiny-house wall or obstacle markers found in {path}")
    return markers


def read_robot(path):
    return RbModel(path, "spawn_extended")


def read_robot_markers(rb, stamp):
    model, data = rb.model, rb.data
    markers = MarkerArray()
    for geom in range(model.ngeom):
        if model.geom_type[geom] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = int(model.geom_dataid[geom])
        mesh_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
        if mesh_name not in CAD_MESH_FILES:
            continue
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id="map")
        marker.ns = "rb_cad"
        marker.id = len(markers.markers)
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        body = int(model.geom_bodyid[geom])
        body_rot = data.xmat[body].reshape(3, 3)
        source_offset = model.geom_pos[geom] - model.mesh_pos[mesh_id]
        pos = data.xpos[body] + body_rot @ source_offset
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = map(float, pos)
        quat = data.xquat[body]
        marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z = map(float, quat)
        scale = model.mesh_scale[mesh_id]
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, scale)
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, model.geom_rgba[geom])
        marker.mesh_resource = (Path(__file__).resolve().parents[4] / "model/assets" / CAD_MESH_FILES[mesh_name]).resolve().as_uri()
        markers.markers.append(marker)
    if not markers.markers:
        raise ValueError("no CAD robot geoms found")
    return markers


CAD_MESH_FILES = {f"m{i}": f"mesh_{i:02d}_00.stl" for i in range(3, 18)}
CAD_MESH_FILES.update({f"{side}_{part}": f"rb_parts/{side}_{part}.stl"
                       for side in ("left", "right") for part in ("body", "thigh", "shin", "wheel", "crank", "coupler")})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="datasets/raw/tof-... session folder")
    parser.add_argument("--scene", type=Path, default=Path(__file__).resolve().parents[4] / "simulation/scenarios/indoor_tiny_house.xml")
    args = parser.parse_args()
    rclpy.init()
    node = MapBridge(args.session.resolve(), args.scene.resolve())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
