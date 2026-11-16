import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from diagnose_tof_scale import diagnose_frame, project


def fixture(scale=.4):
    size=160;focal=size/(2*np.tan(np.deg2rad(22.5)))
    K=[focal,focal,79.5,79.5]
    cal=dict(rotation_matrix=Rotation.from_euler('y',4,degrees=True).as_matrix(),translation_m=[.02,0,-.01])
    y,x=np.indices((size,size))
    ray=np.stack(((x-K[2])/K[0],-(y-K[3])/K[1],-np.ones_like(x)),axis=-1)
    coefficient=-(ray@np.asarray(cal['rotation_matrix']).T)[...,2]
    raw=(1.1-.01)/coefficient/scale
    raw[30:34,30:34]=0;raw[40,40]=np.nan
    z,index,inside=project(raw*scale,K,cal)
    hist=np.tile([1.1,.005],(64,1));valid=np.ones(64,bool)
    rect=np.zeros((64,4))
    for zone in range(64):
        yy,xx=np.nonzero(inside&(index==zone))
        if len(xx):rect[zone]=[yy.min(),xx.min(),yy.max()+1,xx.max()+1]
        else:valid[zone]=False
    return raw,K,cal,hist,rect,valid


class ScaleTests(unittest.TestCase):
    def test_known_global_scale_full_extrinsics_and_missing_pixels(self):
        raw,K,cal,hist,rect,valid=fixture()
        original=raw.copy()
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertTrue(result['fit_accepted'],result['rejection_reasons'])
        self.assertAlmostEqual(result['scale'],.4,places=5)
        self.assertGreaterEqual(result['training_zones'],12)
        self.assertLess(result['candidate']['heldout']['median_abs_m'],1e-5)
        self.assertTrue(np.all(depth[~np.isfinite(raw)|(raw<=.02)]==0))
        np.testing.assert_array_equal(raw,original)
        z,_,inside=project(depth,K,cal)
        self.assertLess(np.max(abs(z[inside&(depth>0)]-1.1)),1e-4)

    def test_scale_outside_fixed_diagnostic_bracket_is_rejected_not_clamped(self):
        raw,K,cal,hist,rect,valid=fixture(scale=.05)
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertFalse(result['fit_accepted'])
        self.assertIsNone(depth)
        self.assertIn('Scale outside fixed diagnostic bracket [0.1, 10]',result['rejection_reasons'])

    def test_mixed_or_too_few_training_zones_reject(self):
        raw,K,cal,hist,rect,valid=fixture()
        hist[:,1]=.1
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertFalse(result['fit_accepted'])
        self.assertIsNone(depth)
        hist[:,1]=.005;valid[:]=False;valid[:16]=True
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertFalse(result['fit_accepted'])
        self.assertIsNone(depth)

    def test_training_cannot_consume_heldout_values(self):
        raw,K,cal,hist,rect,valid=fixture()
        rows,cols=np.divmod(np.arange(64),8)
        hist[(rows+cols)%2==1,0]+=.15
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertAlmostEqual(result['scale'],.4,places=5)
        self.assertFalse(result['fit_accepted'])
        self.assertIsNone(depth)

    def test_control_with_two_metric_depth_regions_is_not_scaled_to_hide_one(self):
        raw,K,cal,hist,rect,valid=fixture()
        # A correct held-out surface must veto a scale inferred from biased training surfaces.
        _,index,inside=project(raw*.4,K,cal)
        raw[inside&(((index//8+index%8)%2)==1)]*=.4
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertFalse(result['fit_accepted'])
        self.assertIsNone(depth)

    def test_predicted_competing_returns_reject_instead_of_filling_zones(self):
        raw,K,cal,hist,rect,valid=fixture()
        raw[:,::2]*=2
        result,depth=diagnose_frame(raw,hist,rect,valid,K,cal)
        self.assertFalse(result['fit_accepted'])
        self.assertIsNone(depth)


if __name__=='__main__':unittest.main()
