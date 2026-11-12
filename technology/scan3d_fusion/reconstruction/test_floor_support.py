import unittest
import numpy as np
from floor_support import FloorSupport


class FloorTests(unittest.TestCase):
    def scene(self):
        h,w=240,320
        K=[290,290,159.5,119.5]
        y,x=np.indices((h,w));rays=np.stack(((x-K[2])/K[0],(y-K[3])/K[1],np.ones_like(x)),-1)
        depth=np.zeros((h,w),np.float32)
        valid=y>145
        depth[valid]=.235/rays[...,1][valid]
        pose=np.eye(4);pose[:3,:3]=[[0,0,1],[-1,0,0],[0,-1,0]];pose[2,3]=.235
        return depth,K,pose,rays

    def test_floor_improves_preserves_obstacle_and_holes(self):
        depth,K,pose,rays=self.scene()
        y,x=np.indices(depth.shape);valid=depth>0
        depth[valid]*=1+.045*np.sin(x[valid]/19)*np.cos(y[valid]/11)
        obstacle=(x>130)&(x<165)&(y>180)&(y<210)
        depth[obstacle]=(.235-.075)/rays[...,1][obstacle]
        depth[215:225,60:85]=0
        original=depth.copy();floor=FloorSupport()
        for i in range(3):
            out,plane,count=floor.apply(depth,K,pose)
            if i<2:self.assertIsNone(plane)
        self.assertIsNotNone(plane);self.assertGreater(count,1000)
        np.testing.assert_array_equal(out[obstacle],original[obstacle])
        self.assertTrue(np.all(out[215:225,60:85]==0))
        mask=(original>0)&~obstacle
        before=.235-original*rays[...,1];after=.235-out*rays[...,1]
        self.assertLess(np.sqrt(np.mean(after[mask]**2)),.8*np.sqrt(np.mean(before[mask]**2)))
        np.testing.assert_allclose(pose[:3,:3]@plane[:3],[0,0,1],atol=1e-6)
        self.assertAlmostEqual(pose[2,3],plane[3])

    def test_depth_tilt_does_not_redefine_gravity(self):
        depth,K,pose,rays=self.scene()
        valid=depth>0
        # A biased depth plane must not tilt the camera or change its height.
        depth[valid]=.24/(rays[...,1][valid]+.025*rays[...,0][valid])
        floor=FloorSupport()
        for _ in range(3):out,plane,count=floor.apply(depth,K,pose)
        self.assertIsNotNone(plane)
        np.testing.assert_allclose(plane,[0,-1,0,.235],atol=1e-7)
        self.assertGreater(count,1000)
        _,missing,_=floor.apply(np.zeros_like(depth),K,pose)
        self.assertIsNone(missing)
        for _ in range(3):_,recovered,_=floor.apply(depth,K,pose)
        np.testing.assert_allclose(recovered,plane)

    def test_wall_or_missing_depth_does_not_become_floor(self):
        _,K,pose,_=self.scene()
        for depth in [np.zeros((240,320)),np.ones((240,320))]:
            floor=FloorSupport()
            for _ in range(4):out,plane,count=floor.apply(depth,K,pose)
            self.assertIsNone(plane);self.assertEqual(count,0)
            np.testing.assert_array_equal(out,depth)

if __name__=='__main__':unittest.main()
