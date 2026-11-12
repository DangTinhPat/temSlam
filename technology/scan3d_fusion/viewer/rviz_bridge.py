#!/usr/bin/env python3
"""Display the exported mesh only; ROS does not participate in reconstruction."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


def sofa_comparison(scene):
    """Display source geometry beside the scan; it never enters reconstruction."""
    root = ET.parse(scene).getroot()
    meshes = {mesh.get("name"): mesh for mesh in root.find("asset").findall("mesh")}
    markers = []
    sofa = next((geom for geom in root.findall("worldbody/geom")
                 if geom.get("name") == "env_mesh_sofa"), None)
    if sofa is None or sofa.get("mesh") not in meshes:
        raise ValueError(f"no sofa mesh in {scene}")
    asset = meshes[sofa.get("mesh")]
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns, marker.id = "original_sofa", 0
    marker.type = Marker.MESH_RESOURCE
    marker.mesh_resource = (scene.parent / asset.get("file")).resolve().as_uri()
    marker.mesh_use_embedded_materials = True
    position = list(map(float, sofa.get("pos", "0 0 0").split()))
    position[0] += 2.6
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
    marker.pose.orientation.w, marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z = (
        map(float, sofa.get("quat", "1 0 0 0").split()))
    marker.scale.x, marker.scale.y, marker.scale.z = map(float, asset.get("scale", "1 1 1").split())
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, sofa.get("rgba").split())
    markers.append(marker)
    center_x, center_y = map(float, sofa.get("pos", "0 0 0").split()[:2])
    for index, (x, text) in enumerate(((center_x, "MESH QUET: RGB + ToF"),
                                        (center_x+2.6, "SOFA GOC: MO PHONG"))):
        label = Marker()
        label.header.frame_id = "map"
        label.ns, label.id = "comparison_labels", index
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y, label.pose.position.z = x, center_y, 1.35
        label.pose.orientation.w = 1.
        label.scale.z = .10
        label.color.r = label.color.g = label.color.b = label.color.a = 1.
        label.text = text
        markers.append(label)
    return markers


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path)
    parser.add_argument("--compare-sofa", action="store_true")
    parser.add_argument("--floor-completion", action="store_true")
    args = parser.parse_args()
    path = args.mesh.resolve(strict=True)
    if path.suffix.lower() not in (".stl", ".dae"):
        parser.error("RViz expects the exported STL or a DAE mesh")
    comparison = sofa_comparison(Path(__file__).resolve().parents[3]/
        "simulation/scenarios/indoor_tiny_house.xml") if args.compare_sofa else []
    rclpy.init()
    node = Node("rgb_tof_clay_mesh")
    publisher = node.create_publisher(Marker, "/rgb_tof/clay_mesh",
        QoSProfile(depth=1+len(comparison), durability=DurabilityPolicy.TRANSIENT_LOCAL))
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "deltar_surface"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.mesh_resource = path.as_uri()
    marker.pose.orientation.w = 1.
    marker.scale.x = marker.scale.y = marker.scale.z = 1.
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = .75, .72, .67, 1.
    floor_layers = []
    if args.floor_completion:
        manifest = json.loads((path.parent/'floor_completion/manifest.json').read_text())
        for name, filename in manifest['layers'].items():
            pub = node.create_publisher(Marker, '/scan3d/'+name,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            layer = Marker()
            layer.header.frame_id = 'map'
            layer.ns = name;layer.id = 0;layer.type = Marker.MESH_RESOURCE
            layer.pose.orientation.w = 1.
            layer.scale.x = layer.scale.y = layer.scale.z = 1.
            layer.mesh_resource = Path(filename).resolve(strict=True).as_uri()
            layer.color.r, layer.color.g, layer.color.b, layer.color.a = (
                (.42,.61,.68,1.) if name == 'inferred' else (.75,.72,.67,1.))
            floor_layers.append((pub,layer))
    def publish():
        stamp = node.get_clock().now().to_msg()
        for item in [marker]+comparison:
            item.header.stamp = stamp
            publisher.publish(item)
        for pub, layer in floor_layers:
            layer.header.stamp = stamp
            pub.publish(layer)
    node.create_timer(1., publish)
    node.get_logger().info(f"Displaying reconstructed mesh: {path}")
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
