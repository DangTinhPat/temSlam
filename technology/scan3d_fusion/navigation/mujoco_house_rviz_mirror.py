"""Show the RViz navigation pose as the RB CAD model in the full MuJoCo house.

Kinematic visualization only: RViz publishes the pose; MuJoCo physics is not stepped.
"""
import argparse
import math
from pathlib import Path

import mujoco
import mujoco.viewer
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MODEL = ROOT / 'simulation/scenarios/indoor_tiny_house_living_room.xml'
# Fit from synchronized camera poses and the MuJoCo capture trace for map v10.
MAP_TO_SIM_YAW = math.radians(8.41194261184)
MAP_TO_SIM_XY = (-0.57168932, -0.07285786)


class PoseMirror(Node):
    def __init__(self, model, data, initial_pose):
        super().__init__('rb_mujoco_house_pose_mirror')
        self.data = data
        self.joint_qpos = int(model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, 'root')])
        self.pose = initial_pose
        self.create_subscription(PoseStamped, '/rb_navigation/pose', self.pose_callback, 10)

    def pose_callback(self, message):
        p, q = message.pose.position, message.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.z, q.w)):
            return
        self.pose = (p.x, p.y, 2*math.atan2(q.z, q.w))

    def apply(self):
        x, y, yaw = self.pose
        c, s = math.cos(MAP_TO_SIM_YAW), math.sin(MAP_TO_SIM_YAW)
        q = self.data.qpos[self.joint_qpos:self.joint_qpos+7]
        q[:2] = (c*x - s*y + MAP_TO_SIM_XY[0], s*x + c*y + MAP_TO_SIM_XY[1])
        yaw += MAP_TO_SIM_YAW
        q[3:] = math.cos(yaw/2), 0., 0., math.sin(yaw/2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--initial-pose', nargs=3, type=float, default=(.7, -1.6, 0.),
                        metavar=('X', 'Y', 'YAW_DEG'))
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f'MuJoCo house model not found: {args.model}')
    if not all(math.isfinite(v) for v in args.initial_pose):
        parser.error('Initial pose must be finite')
    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'root')
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'spawn_extended')
    if root < 0 or model.jnt_type[root] != mujoco.mjtJoint.mjJNT_FREE or key < 0:
        parser.error('House model must define a freejoint named root and spawn_extended keyframe')
    mujoco.mj_resetDataKeyframe(model, data, key)
    rclpy.init()
    node = PoseMirror(model, data, (args.initial_pose[0], args.initial_pose[1],
                                    math.radians(args.initial_pose[2])))
    node.apply()
    mujoco.mj_forward(model, data)
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = (0., 0., .8)
                viewer.cam.distance = 10.
                viewer.cam.azimuth = 135.
                viewer.cam.elevation = -35.
            node.get_logger().info(
                'Full MuJoCo house open; applying map-v10 → MuJoCo alignment and '
                'following /rb_navigation/pose (kinematic mirror).')
            while viewer.is_running():
                rclpy.spin_once(node, timeout_sec=.02)
                with viewer.lock():
                    node.apply()
                    mujoco.mj_forward(model, data)
                viewer.sync()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
