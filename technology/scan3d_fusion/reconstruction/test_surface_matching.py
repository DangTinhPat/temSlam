import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial import cKDTree
from surface_matching import match_surface, planar_change,refine_sequence


class SurfaceMatchingTests(unittest.TestCase):
    def test_refinement_bounds_each_pose_against_its_original_odometry(self):
        with tempfile.TemporaryDirectory() as directory:
            session=Path(directory);frames=[]
            for i in range(8):
                file=f'{i}.npz';np.savez(session/file,depth=np.full((16,16),1000,np.uint16))
                frames.append(dict(file=file,time_s=float(i),camera_to_world=np.eye(4).tolist()))
            # Model a repeated small matching bias; it must not accumulate across frames.
            def match(*a):return planar_change([.05,0,0],a[-1]),dict(accepted=True,matches=100,delta=[.05,0,0])
            with patch('surface_matching.vertical_points',return_value=(np.zeros((1,3)),np.array([[1.,0,0]]))), \
                    patch('surface_matching.match_surface',side_effect=match):
                refined,_=refine_sequence(session,dict(frames=frames,intrinsics=[10,10,8,8]))
            for f in refined['frames']:
                self.assertAlmostEqual(f['camera_to_world'][0][3],.05)
            self.assertEqual(frames[-1]['camera_to_world'][0][3],0.,'Preserve original history')

    def room_corner(self):
        rng=np.random.default_rng(4)
        points=rng.uniform([0,0,.2],[2,2,1.6],(1500,3))
        points[:750,0]=0;points[750:,1]=0
        normals=np.zeros_like(points);normals[:750,0]=1;normals[750:,1]=1
        return points,normals

    def test_recovers_planar_error_with_depth_noise(self):
        target,normals=self.room_corner();origin=np.array([.7,.6,.235])
        error=planar_change([.06,-.04,.025],origin)
        source=target@error[:3,:3].T+error[:3,3]
        source+=np.random.default_rng(8).normal(0,.005,source.shape)
        change,report=match_surface(source,normals@error[:3,:3].T,target,normals,cKDTree(target),origin)
        self.assertTrue(report['accepted'])
        residual=change@error
        self.assertLess(np.linalg.norm(residual[:3,3]),.01)
        np.testing.assert_allclose(residual[:3,:3],np.eye(3),atol=.005)
        self.assertEqual(change[2,3],0)
        np.testing.assert_array_equal(change[2,:3],[0,0,1])

    def test_single_wall_or_no_overlap_keeps_odometry(self):
        target,normals=self.room_corner()
        for source,n,reference,rn in [(target[:750]+[.03,0,0],normals[:750],target[:750],normals[:750]),
                                      (target+10,normals,target,normals)]:
            change,report=match_surface(source,n,reference,rn,cKDTree(reference),np.zeros(3))
            self.assertFalse(report['accepted'])
            np.testing.assert_array_equal(change,np.eye(4))


if __name__=='__main__':unittest.main()
