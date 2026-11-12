import unittest
import numpy as np
from depth_support import tof_consistency_mask


class DepthSupportTests(unittest.TestCase):
    def test_own_zone_rejects_far_return_and_invalid_zone(self):
        calibration={'rotation_matrix':np.eye(3),'translation_m':np.zeros(3)}
        K=[20,20,9.5,9.5]
        hist=np.tile([2.,.01],(64,1));hist[36]=[1.,.01]
        valid=np.ones(64,bool)
        # The centre ray belongs to zone 36; far neighbours must not admit a 2 m point.
        self.assertFalse(tof_consistency_mask(np.full((20,20),2.),hist,valid,K,calibration)[10,10])
        self.assertTrue(tof_consistency_mask(np.ones((20,20)),hist,valid,K,calibration)[10,10])
        valid[36]=False
        self.assertFalse(tof_consistency_mask(np.ones((20,20)),hist,valid,K,calibration)[10,10])

    def test_broad_distribution_is_not_unlimited_support(self):
        calibration={'rotation_matrix':np.eye(3),'translation_m':np.zeros(3)}
        hist=np.tile([1.,2.],(64,1));valid=np.ones(64,bool);K=[20,20,9.5,9.5]
        mask=tof_consistency_mask(np.full((20,20),2.),hist,valid,K,calibration)
        self.assertFalse(mask.any())
        mask=tof_consistency_mask(np.full((20,20),1.03),hist,valid,K,calibration)
        self.assertTrue(mask[10,10]);self.assertFalse(mask[10,0])
        depth=np.ones((20,20));depth[10,10]=np.nan
        self.assertFalse(tof_consistency_mask(depth,hist,valid,K,calibration)[10,10])

    def test_projected_near_return_blocks_far_zone(self):
        calibration={'rotation_matrix':np.eye(3),'translation_m':np.zeros(3)}
        hist=np.tile([2.,.01],(64,1));hist[0]=[1.,.01]
        rect=np.tile([0,0,20,20],(64,1));depth=np.full((20,20),2.)
        mask=tof_consistency_mask(depth,hist,np.ones(64,bool),[20,20,9.5,9.5],calibration,rect)
        self.assertFalse(mask.any())


if __name__=='__main__':unittest.main()
