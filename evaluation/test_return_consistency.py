import math
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'technology/scan3d_fusion/capture'))
from capture_mujoco import zone_statistics, closest_cluster
from diagnose_return_consistency import project_rays, diagnose_frame, selected_return_slope_support, diagnostic_consistency_mask, fit_original_surface, surface_visibility_reasons, strict_visibility_reasons

SENSOR=dict(min_range_m=.02,max_range_m=4.,noise=dict(near_accuracy_m=.02,far_accuracy_fraction=.05,sigma_fraction_of_bound=0.))
IDENTITY=dict(rotation_matrix=np.eye(3).tolist(),translation_m=[0,0,0])
HALF=np.tan(np.deg2rad(22.5))
K=[64/HALF,64/HALF,63.5,63.5]

def original_zone_statistics(depth, sensor, rng):
    """Approximate a zone return with many rays, in ToF optical-Z metres."""
    if depth.shape != (128, 128):
        raise ValueError("ToF sampling image must be 128 x 128")
    hist = np.zeros((64, 2), np.float32)
    valid = np.zeros(64, bool)
    for row in range(8):
        for col in range(8):
            values = depth[row*16:(row+1)*16, col*16:(col+1)*16].ravel()
            values = values[np.isfinite(values) & (values >= sensor["min_range_m"]) &
                            (values <= sensor["max_range_m"])]
            if len(values) < 32:
                continue
            # shortcut: equal-area rays approximate a return distribution; no photon/reflectance model.
            counts, edges = np.histogram(values, bins=np.arange(0, 4.04, .04))
            occupied = np.flatnonzero(counts >= max(3, counts.max() * .1))
            if not len(occupied):
                continue
            clusters = np.split(occupied, np.flatnonzero(np.diff(occupied) > 1) + 1)
            # The RB sensor is configured for the closest significant target.
            cluster = next((c for c in clusters if counts[c].sum() >= len(values)*.2), None)
            if cluster is None:
                continue
            selected = values[(values >= edges[cluster[0]]) & (values <= edges[cluster[-1]+1])]
            mean = float(selected.mean())
            noise = sensor["noise"]
            bound = float(noise["near_accuracy_m"]) if mean <= .2 else mean*float(noise["far_accuracy_fraction"])
            sigma_noise = bound * float(noise["sigma_fraction_of_bound"])
            index = row*8+col
            hist[index] = [np.clip(mean+rng.normal(0, sigma_noise), .02, 4),
                           max(.005, math.hypot(float(selected.std()), sigma_noise))]
            valid[index] = True
    return hist, valid


class ReturnTests(unittest.TestCase):
    def test_extraction_bit_exact_and_rng_calls(self):
        rng=np.random.default_rng(8);depth=rng.uniform(.02,4,(128,128));depth[:16,:16]=1
        depth[16:32,:16]=np.nan;depth[32:48,:16]=0
        depth[48:64,:16]=np.tile(np.r_[np.full(4,.6),np.full(12,1.4)],(16,1))
        for noise in (0.,.2):
            sensor=dict(SENSOR,noise=dict(SENSOR['noise'],sigma_fraction_of_bound=noise))
            a=np.random.default_rng(2026);b=np.random.default_rng(2026)
            old,valid=original_zone_statistics(depth,sensor,a);new,newvalid=zone_statistics(depth,sensor,b)
            np.testing.assert_array_equal(old,new);np.testing.assert_array_equal(valid,newvalid)
            self.assertEqual(a.bit_generator.state,b.bit_generator.state)

    def test_exact_front_plane_zero_and_biased_plane_recovery(self):
        true=np.ones((128,128));hist,valid=zone_statistics(true,SENSOR,np.random.default_rng(1))
        rect=np.tile([0,0,128,128],(64,1))
        result,candidate=diagnose_frame(true,hist,rect,valid,K,IDENTITY,bias_enabled=True)
        np.testing.assert_array_equal(candidate,true)
        self.assertTrue(all(z['proposed_bias_m']==0 for z in result['zones']))
        _,no_bias=diagnose_frame(true+.04,hist,rect,valid,K,IDENTITY)
        np.testing.assert_array_equal(no_bias,true+.04)
        result,candidate=diagnose_frame(true+.04,hist,rect,valid,K,IDENTITY,bias_enabled=True)
        np.testing.assert_allclose(candidate,true,atol=.001)
        self.assertTrue(all(z['candidate_accepted'] for z in result['zones']))

    def test_oblique_true_geometry_is_not_shifted_to_whole_mean(self):
        y=np.arange(128)[:,None];ry=((y+.5)/64-1)*HALF
        depth=np.broadcast_to(np.divide(.24,ry,where=ry!=0),(128,128)).copy()
        depth[:80]=0;depth[96:]=0
        hist,valid=zone_statistics(depth,SENSOR,np.random.default_rng(1))
        info=closest_cluster(depth[80:96,32:48].ravel(),SENSOR)
        self.assertAlmostEqual(info['whole_mean_m'],1.606186,places=5)
        self.assertAlmostEqual(info['mean_m'],1.283162,places=5)
        result,candidate=diagnose_frame(depth,hist,np.tile([0,0,128,128],(64,1)),valid,K,IDENTITY,bias_enabled=True)
        np.testing.assert_array_equal(candidate,depth)
        self.assertEqual(result['zones'][42]['proposed_bias_m'],0)

    def test_two_surfaces_nearest_significance_and_no_zone_shift(self):
        values=np.r_[np.full(64,.6),np.full(192,1.4)]
        self.assertAlmostEqual(closest_cluster(values,SENSOR)['mean_m'],.6)
        values[:32]=1.4
        self.assertAlmostEqual(closest_cluster(values,SENSOR)['mean_m'],1.4)
        depth=np.ones((128,128));depth[:16,:16]=np.r_[np.full(64,.64),np.full(192,1.44)].reshape(16,16)
        measured=depth.copy();measured[:16,:16]-=.04
        hist,valid=zone_statistics(measured,SENSOR,np.random.default_rng(1))
        result,candidate=diagnose_frame(depth,hist,np.tile([0,0,128,128],(64,1)),valid,K,IDENTITY,bias_enabled=True)
        self.assertFalse(result['zones'][0]['candidate_accepted'])
        np.testing.assert_array_equal(candidate[:16,:16],depth[:16,:16])

    def test_oblique_selected_return_slope_acceptance_legacy_rejects(self):
        from depth_support import sloping_zone_support
        Kactual=[579.4112549695428]*2+[319.5,239.5]
        yy,xx=np.indices((480,640));ray_y=(yy-Kactual[3])/Kactual[1]
        depth=np.zeros((480,640));np.divide(.24,ray_y,out=depth,where=ray_y>0)
        depth[(depth<.02)|(depth>4)]=0
        raster=np.zeros((128,128));ry=((np.arange(128)+.5)/64-1)*HALF
        raster[80:96]=np.broadcast_to((.24/ry[80:96])[:,None],(16,128))
        hist,valid=zone_statistics(raster,SENSOR,np.random.default_rng(1))
        projected=project_rays(depth,Kactual,IDENTITY);z=projected['z']
        safe=np.where(z>.02,z,1);u=((xx-Kactual[2])*depth/Kactual[0]/safe/HALF+1)*4
        v=(1+(yy-Kactual[3])*depth/Kactual[1]/safe/HALF)*4
        inside=(depth>.02)&(u>=0)&(u<8)&(v>=0)&(v<8)
        index=np.floor(np.where(inside,v,0)).astype(int)*8+np.floor(np.where(inside,u,0)).astype(int)
        legacy=sloping_zone_support(z,u,v,index,inside,hist,valid)
        semantic=selected_return_slope_support(z,u,v,index,inside,hist,valid)
        target=inside&(index==42)
        self.assertEqual(np.count_nonzero(legacy&target),0)
        self.assertGreater(np.count_nonzero(semantic&target),100)
        # The pointwise nearest-return envelope is unchanged; far tail stays out.
        self.assertFalse(np.any(semantic&target&(abs(z-hist[42,0])>max(.08,3*hist[42,1]))))

    def test_diagnostic_legacy_mask_exact_and_biased_oblique_far_tail_unchanged(self):
        from depth_support import tof_consistency_mask
        y=np.arange(128);ry=((y+.5)/64-1)*HALF
        depth=np.zeros((128,128));depth[80:96]=np.broadcast_to((.24/ry[80:96])[:,None],(16,128))
        hist,valid=zone_statistics(depth,SENSOR,np.random.default_rng(1));rect=np.tile([0,0,128,128],(64,1))
        np.testing.assert_array_equal(diagnostic_consistency_mask(depth,hist,valid,K,IDENTITY,rect),tof_consistency_mask(depth,hist,valid,K,IDENTITY,rect))
        biased=np.where(depth>.02,depth+.04,depth)
        result,candidate=diagnose_frame(biased,hist,rect,valid,K,IDENTITY,bias_enabled=True)
        projected=project_rays(biased,K,IDENTITY)
        far=np.zeros_like(depth,dtype=bool)
        for zone in range(64):
            info=result['zones'][zone]['baseline']
            if 'high_m' not in info:continue
            row,col=divmod(zone,8);cells=projected['cell_by_pixel']
            far|=(cells>=0)&((cells//128)//16==row)&((cells%128)//16==col)&(projected['z']>info['high_m'])
        self.assertGreater(far.sum(),0)
        np.testing.assert_array_equal(candidate[far],biased[far])
        np.testing.assert_array_equal(candidate[biased==0],biased[biased==0])

    def test_actual_calibration_biased_plane_exposes_owner_gate(self):
        import json
        cal=json.loads((ROOT/'technology/scan3d_fusion/config/sim_calibration.json').read_text())['camera_to_tof']
        Kactual=[579.4112549695428,579.4112549695428,319.5,239.5]
        y,x=np.indices((480,640));R=np.asarray(cal['rotation_matrix']);t=np.asarray(cal['translation_m'])
        rays=np.stack(((x-Kactual[2])/Kactual[0],-(y-Kactual[3])/Kactual[1],-np.ones_like(x)),axis=-1)
        dz=-(rays@R.T)[...,2]
        raw=(1.04+t[2])/dz
        hist,valid=zone_statistics(np.ones((128,128)),SENSOR,np.random.default_rng(1))
        report,candidate=diagnose_frame(raw,hist,np.tile([0,0,480,640],(64,1)),valid,Kactual,cal,bias_enabled=True)
        proposals=[z for z in report['zones'] if 'proposal' in z]
        self.assertGreater(len(proposals),0)
        # Preserve the frozen conservative gate. This is an expected limitation,
        # not authorization to weaken it against measured scene scores.
        self.assertTrue(all('occlusion/source ownership changed' in z['rejection_reasons'] for z in proposals))
        self.assertFalse(any(z['candidate_accepted'] for z in report['zones']))
        np.testing.assert_array_equal(candidate,raw)
        with self.assertRaisesRegex(ValueError,'25mm layers not identifiable'):
            diagnose_frame(raw,hist,np.tile([0,0,480,640],(64,1)),valid,Kactual,cal,bias_enabled=True,owner_policy='surface')

    def test_improving_two_layer_owner_swap_in_neighbor_is_rejected(self):
        # Each ToF cell sees two RGB samples: selected near plane and hidden rear.
        raw=np.empty((128,256));raw[:,::2]=1.04;raw[:,1::2]=2.04
        intrinsics=[128/HALF,64/HALF,127.5,63.5]
        before=project_rays(raw,intrinsics,IDENTITY)
        info=closest_cluster(before['raster'][80:96,32:48],SENSOR)
        surface=fit_original_surface(before,42,info)
        self.assertIsNotNone(surface)
        proposal=raw.copy();proposal[:,::2]=1.0
        # Swap visible foreground and hidden background in NEIGHBOR zone43.
        proposal[84,104]=2.04;proposal[84,105]=1.0
        after=project_rays(proposal,intrinsics,IDENTITY)
        self.assertLess(abs(after['raster'].mean()-1),abs(before['raster'].mean()-1))
        reasons=surface_visibility_reasons(before,after,surface['member'])
        self.assertIn('changed owner outside original continuous surface',reasons)
        self.assertIn('original selected-surface visibility changed',reasons)
        # A same-surface resampling exception must not turn into an owner bypass.
        self.assertFalse(surface['member'][84,105])

    def test_25mm_neighbor_layer_swap_exposes_nonidentifiability(self):
        raw=np.empty((128,256));raw[:,::2]=1.04;raw[:,1::2]=1.065
        intrinsics=[128/HALF,64/HALF,127.5,63.5]
        before=project_rays(raw,intrinsics,IDENTITY)
        info=closest_cluster(before['raster'][80:96,32:48],SENSOR)
        surface=fit_original_surface(before,42,info)
        self.assertIsNotNone(surface)
        proposal=raw.copy();proposal[:,::2]=1.0
        proposal[84,104]=1.065;proposal[84,105]=1.0
        after=project_rays(proposal,intrinsics,IDENTITY)
        self.assertLess(abs(after['raster'].mean()-1),abs(before['raster'].mean()-1))
        reasons=surface_visibility_reasons(before,after,surface['member'])
        # This is the FAILED proposed surface predicate, retained as evidence.
        # The corrected-depth API rejects this policy entirely. Strict source
        # identity remains active, so neither measured nor fitted moments can
        # authorize this hidden-layer substitution.
        self.assertEqual(reasons,[])
        np.testing.assert_array_equal(before['owner'][80:96,32:48],after['owner'][80:96,32:48])
        self.assertIn('occlusion/source ownership changed',strict_visibility_reasons(before,after))
        self.assertTrue(surface['member'][84,105])
        self.assertNotEqual(before['owner'][84,52],after['owner'][84,52])
        hist,valid=zone_statistics(np.ones((128,128)),SENSOR,np.random.default_rng(1))
        with self.assertRaisesRegex(ValueError,'25mm layers not identifiable'):
            diagnose_frame(raw,hist,np.tile([0,0,128,256],(64,1)),valid,intrinsics,IDENTITY,bias_enabled=True,owner_policy='surface')

    def test_frozen_variant_generation_preserves_invalid_and_expected_bias(self):
        from run_return_variants import generate
        true=np.ones((128,128),dtype=np.float32)
        hist,valid=zone_statistics(true,SENSOR,np.random.default_rng(1));rect=np.tile([0,0,128,128],(64,1))
        raw=true+.04
        # Missing rays disqualify only their zone; never synthesize a pixel.
        raw[0,0]=0
        for variant in ('baseline','selected_bias_legacy_slope','no_bias_selected_slope','selected_bias_selected_slope'):
            report,candidate,supported=generate(raw,hist,rect,valid,K,IDENTITY,variant)
            self.assertEqual(candidate[0,0],0);self.assertEqual(supported[0,0],0)
            if variant=='no_bias_selected_slope':np.testing.assert_array_equal(candidate,raw)
            if variant.startswith('selected_bias'):
                self.assertGreater(sum(z['candidate_accepted'] for z in report['zones']),0)
                self.assertAlmostEqual(candidate[32,32],1.,places=5)

    def test_holes_invalid_zone_and_zbuffer_nonzero_baseline(self):
        depth=np.ones((128,128));depth[2,2]=0;depth[3,3]=np.nan
        hist,valid=zone_statistics(np.ones_like(depth),SENSOR,np.random.default_rng(1));valid[63]=False
        result,candidate=diagnose_frame(depth,hist,np.tile([0,0,128,128],(64,1)),valid,K,IDENTITY,bias_enabled=True)
        self.assertIn('incomplete angular coverage',result['zones'][0]['rejection_reasons'])
        np.testing.assert_array_equal(np.isfinite(candidate)&(candidate>.02),np.isfinite(depth)&(depth>.02))
        # Parallel optical axes, 2cm baseline: two RGB samples hit one ToF cell;
        # near sample owns it and the hidden rear source must never enter moments.
        cal=dict(rotation_matrix=np.eye(3),translation_m=[.02,0,0])
        raw=np.zeros((1,3));raw[0,:2]=[1.,2.]
        projected=project_rays(raw,[64,64,0.,0.],cal)
        self.assertEqual(np.isfinite(projected['raster']).sum(),1)
        self.assertEqual(projected['owner'][np.isfinite(projected['raster'])].item(),0)
        self.assertEqual(projected['raster'][np.isfinite(projected['raster'])].item(),1)

if __name__=='__main__':unittest.main()
