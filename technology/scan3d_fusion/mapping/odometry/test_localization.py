import unittest
import json
import numpy as np
from scipy.spatial.transform import Rotation
from localization import BodyLocalization, odom_body

class LocalizationTest(unittest.TestCase):
    def packet(self, t, x=0):
        return dict(time_s=t,x=x,y=0,yaw=0,pitch=0,roll=0,axle_body_x=.02,
                    axle_body_z=-.15,wheel_radius=.035,valid=1,slip=False)

    def test_rotating_head_and_visual_loss(self):
        fusion=BodyLocalization()
        packet=self.packet(0)
        world=np.eye(4);world[:3,3]=[4,2,0]
        for i in range(4):
            packet=self.packet(i*.1)
            mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('z',i*.6).as_matrix()
            mount[:3,3]=mount[:3,:3]@[.09,.02,.1]
            camera=world@odom_body(packet)@mount
            out=fusion.update(i*100_000_000,packet,mount,camera)
            json.dumps(out)
            np.testing.assert_allclose(out['world_from_body'],world@odom_body(packet),atol=1e-12)
        packet=self.packet(.4,.1)
        out=fusion.update(400_000_000,packet,mount,None)
        np.testing.assert_allclose(out['world_from_body'],world@odom_body(packet),atol=1e-12)
        self.assertFalse(out['visual_accepted'])
        camera[:3,3]+=100
        out=fusion.update(500_000_000,self.packet(.5,.1),mount,camera)
        self.assertFalse(out['visual_accepted'])

    def test_sync_and_support(self):
        fusion=BodyLocalization()
        with self.assertRaises(ValueError): fusion.update(100_000_000,self.packet(0),np.eye(4))
        with self.assertRaises(ValueError): fusion.update(float('nan'),self.packet(0),np.eye(4))
        packet=self.packet(.1);packet['valid']=0
        self.assertIsNone(fusion.update(100_000_000,packet,np.eye(4)))

    def test_level_floor_removes_visual_map_tilt_and_height_drift(self):
        fusion=BodyLocalization(level_floor=True)
        tilted=np.eye(4)
        tilted[:3,:3]=Rotation.from_euler('xy',[.08,-.06]).as_matrix()
        tilted[:3,3]=[2,3,-.25]
        for i in range(5):
            packet=self.packet(i*.1,i*.2)
            visual=tilted@odom_body(packet)
            visual[2,3]+=.02*i
            out=fusion.update(i*100_000_000,packet,np.eye(4),visual)
            body=np.asarray(out['world_from_body'])
            self.assertAlmostEqual(body[2,3],.185)
            np.testing.assert_allclose(body[:3,2],[0,0,1],atol=1e-10)

if __name__=='__main__': unittest.main()
