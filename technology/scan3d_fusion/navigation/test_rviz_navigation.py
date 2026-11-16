import math
from pathlib import Path
import unittest
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,PointStamped
from grid import NavigationGrid,FREE,OCCUPIED,UNKNOWN
from rviz_navigation import NavigationNode,grid_message,read_pose


def pose_message(x,y,yaw=0.,initial=False,frame='map'):
    msg=PoseWithCovarianceStamped() if initial else PoseStamped()
    msg.header.frame_id=frame
    pose=msg.pose.pose if initial else msg.pose
    pose.position.x=float(x);pose.position.y=float(y)
    pose.orientation.z=math.sin(yaw/2);pose.orientation.w=math.cos(yaw/2)
    return msg


class ConversionTests(unittest.TestCase):
    def test_pose_frame_and_quaternion_validation(self):
        np.testing.assert_allclose(read_pose(pose_message(1,2,.7)),[1,2,.7])
        for msg in [pose_message(1,2,frame='odom'),pose_message(float('nan'),2)]:
            with self.assertRaises(ValueError):read_pose(msg)
        msg=pose_message(1,2);msg.pose.orientation.w=0.
        with self.assertRaises(ValueError):read_pose(msg)

    def test_grid_costs_order_and_crop_coordinates(self):
        cells=np.zeros((20,20),np.int8);cells[10,10]=OCCUPIED;cells[5,5]=UNKNOWN
        grid=NavigationGrid(cells,[-2,-3],.1,.25,demo_only=True)
        raw=grid_message(grid)
        self.assertEqual(raw.data[10*20+10],100);self.assertEqual(raw.data[5*20+5],-1)
        cost=grid_message(grid,inflated=True)
        self.assertEqual(cost.data[10*20+11],99);self.assertEqual(cost.info.resolution,.1)
        crop=grid_message(grid,inflated=True,crop=(4,6,9,12))
        self.assertEqual((crop.info.width,crop.info.height),(5,6))
        self.assertAlmostEqual(crop.info.origin.position.x,-1.6)
        self.assertAlmostEqual(crop.info.origin.position.y,-2.4)
        np.testing.assert_equal(np.array(crop.data).reshape(6,5),np.array(cost.data).reshape(20,20)[6:12,4:9])


class NavigationInputTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):rclpy.init()
    @classmethod
    def tearDownClass(cls):rclpy.shutdown()
    def setUp(self):
        self.grid=NavigationGrid(np.zeros((80,80),np.int8),[0,0],.05,.25,demo_only=True)
        self.node=NavigationNode(self.grid,Path('/tmp/not-rendered.stl'),[1,1,0])
    def tearDown(self):self.node.destroy_node()

    def test_goal_reaches_position_and_requested_heading(self):
        self.node.goal_callback(pose_message(1.6,1.3,.7))
        now=self.node.last_tick
        for _ in range(1000):
            now+=.05;self.node.step(now)
            if self.node.navigator.state=='reached':break
        self.assertEqual(self.node.navigator.state,'reached')
        self.assertLess(np.linalg.norm(self.node.pose[:2]-[1.6,1.3]),.025)
        self.assertLess(abs(self.node.pose[2]-.7),.04)
        self.assertGreater(len(self.node.trail),2)

    def test_drop_pose_cancels_old_goal_and_clears_old_paths(self):
        self.node.goal_callback(pose_message(2,2))
        self.node.pose_callback(pose_message(1.4,1.4,1.,initial=True))
        self.assertEqual(self.node.navigator.state,'idle')
        self.assertEqual(self.node.navigator.path,[])
        self.assertEqual(len(self.node.trail),1)
        np.testing.assert_allclose(self.node.pose,[1.4,1.4,1.])

    def test_invalid_pose_or_goal_stops_without_teleporting(self):
        for callback,msg in [(self.node.pose_callback,pose_message(0,0,initial=True)),
                             (self.node.goal_callback,pose_message(2,2,frame='odom'))]:
            self.node.goal_callback(pose_message(2,2));callback(msg)
            self.assertNotIn(self.node.navigator.state,('planning','driving','aligning'))
            np.testing.assert_allclose(self.node.pose,[1,1,0])

    def test_stop_click_latches_until_new_goal(self):
        self.node.goal_callback(pose_message(2,2));self.node.stop_callback(PointStamped())
        now=self.node.last_tick
        for i in range(3):self.node.step(now+.05*(i+1))
        self.assertEqual(self.node.navigator.state,'cancelled')
        np.testing.assert_allclose(self.node.pose,[1,1,0])


if __name__=='__main__':unittest.main()
