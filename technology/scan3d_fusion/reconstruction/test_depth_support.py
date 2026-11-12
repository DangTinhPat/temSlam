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

    def test_sloping_surface_keeps_range_variation_without_filling_holes(self):
        focal=160/(2*np.tan(np.deg2rad(22.5)));K=[focal,focal,79.5,79.5]
        y,x=np.indices((160,160));depth=np.zeros((160,160))
        np.divide(.2*focal,y-79.5,out=depth,where=y>79.5)
        depth[depth>4]=0
        hist=np.zeros((64,2));valid=np.zeros(64,bool)
        for r in range(8):
            for c in range(8):
                values=depth[r*20:(r+1)*20,c*20:(c+1)*20]
                if np.all(values>0):
                    hist[r*8+c]=[values.mean(),values.std()];valid[r*8+c]=True
        calibration={'rotation_matrix':np.eye(3),'translation_m':np.zeros(3)}
        mask=tof_consistency_mask(depth,hist,valid,K,calibration)
        zone=5*8+4
        self.assertGreater(abs(depth[100,90]-hist[zone,0]),.15)
        self.assertTrue(mask[100,90])
        self.assertTrue(mask[100:120,80:100].all())
        depth[110,90]=0;depth[111,90]=np.nan;depth[112,90]+=1
        mask=tof_consistency_mask(depth,hist,valid,K,calibration)
        self.assertFalse(mask[110,90]);self.assertFalse(mask[111,90]);self.assertFalse(mask[112,90])

    def test_sloping_surface_with_wrong_tof_statistics_is_not_extended(self):
        focal=160/(2*np.tan(np.deg2rad(22.5)));K=[focal,focal,79.5,79.5]
        y,x=np.indices((160,160));depth=np.zeros((160,160))
        np.divide(.2*focal,y-79.5,out=depth,where=y>79.5)
        hist=np.tile([.3,.3],(64,1))
        mask=tof_consistency_mask(depth,hist,np.ones(64,bool),K,
            {'rotation_matrix':np.eye(3),'translation_m':np.zeros(3)})
        self.assertFalse(mask[100:120,80:100].any())


if __name__=='__main__':unittest.main()
