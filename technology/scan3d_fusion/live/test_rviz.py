"""Late RViz subscribers need the saved cloud and trajectory without a new scan."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np
try:
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from visualization_msgs.msg import Marker
except ImportError:
    rclpy = None


@unittest.skipIf(rclpy is None, 'ROS display integration requires the ROS environment')
class SavedDisplayTest(unittest.TestCase):
    def test_late_viewer_receives_saved_trajectory_and_cloud(self):
        with tempfile.TemporaryDirectory() as tmp:
            np.savez(Path(tmp)/'cloud.npz', xyz=np.array([[1., 2., 3.]], np.float32),
                     path=np.array([[0., 0., 0.], [1., 0., 0.]]))
            context = Context()
            rclpy.init(context=context, domain_id=153)
            node = rclpy.create_node('saved_display_test', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            child = subprocess.Popen([sys.executable, str(Path(__file__).with_name('rviz.py')), tmp],
                                     env=dict(os.environ, ROS_DOMAIN_ID='153'),
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                status = []
                node.create_subscription(Marker, '/scan3d/info', lambda m: status.append(m.ns), 10)
                deadline = time.monotonic()+10
                while status.count('status')<12 and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.1)
                self.assertGreaterEqual(status.count('status'),12, 'Wait beyond the status history depth')
                received = {}
                qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                def on_info(message):
                    if message.ns == 'trajectory': received['trajectory'] = len(message.points)
                node.create_subscription(Marker, '/scan3d/trajectory', on_info, qos)
                node.create_subscription(PointCloud2, '/scan3d/cloud',
                                         lambda m: received.update(cloud=m.width), qos)
                deadline = time.monotonic()+3
                while len(received)<2 and time.monotonic()<deadline:
                    executor.spin_once(timeout_sec=.1)
                self.assertEqual(received, {'trajectory': 2, 'cloud': 1})
            finally:
                child.terminate()
                child.wait(timeout=5)
                executor.shutdown()
                node.destroy_node()
                rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
