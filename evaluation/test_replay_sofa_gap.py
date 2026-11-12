import unittest
import numpy as np
from replay_sofa_gap import anchored_pose,steady_sweep


class AnchoredPoseTests(unittest.TestCase):
    def test_room_selection_only_accepts_stable_forward_sweeps(self):
        a=dict(time_s=1.,x=0.,y=0.,head_yaw=0.,valid=True)
        b=dict(a,time_s=1.1,head_yaw=.035)
        self.assertTrue(steady_sweep(a,b))
        for changed in [dict(x=.1),dict(head_yaw=-.035),dict(valid=False),dict(time_s=2),dict(time_s=1)]:
            self.assertFalse(steady_sweep(a,dict(b,**changed)))

    def test_anchor_is_preserved_and_delta_comes_from_local_odometry(self):
        local=np.eye(4);local[:3,3]=[10,20,.24]
        mapped=np.eye(4);mapped[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];mapped[:3,3]=[-2,-3,.24]
        np.testing.assert_allclose(anchored_pose(mapped,local,local),mapped)
        current=local.copy();current[0,3]+=.1
        expected=mapped.copy();expected[1,3]+=.1
        np.testing.assert_allclose(anchored_pose(mapped,local,current),expected,atol=1e-12)

    def test_head_rotation_changes_camera_orientation_without_map_translation_injection(self):
        anchor=np.eye(4);anchor[:3,3]=[-2,-2,.24]
        current=np.eye(4);current[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        result=anchored_pose(anchor,np.eye(4),current)
        np.testing.assert_allclose(result[:3,3],anchor[:3,3])
        np.testing.assert_allclose(result[:3,:3],current[:3,:3])


if __name__=='__main__':unittest.main()
