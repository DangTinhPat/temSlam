#!/usr/bin/env python3
"""Display the exported mesh only; ROS does not participate in reconstruction."""
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", type=Path)
    args = parser.parse_args()
    path = args.mesh.resolve(strict=True)
    if path.suffix.lower() not in (".stl", ".dae"):
        parser.error("RViz expects the exported STL or a DAE mesh")
    rclpy.init()
    node = Node("rgb_tof_clay_mesh")
    publisher = node.create_publisher(Marker, "/rgb_tof/clay_mesh",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
    def publish():
        marker.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(marker)

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
