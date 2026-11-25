"""CPU regressions for the isolated wheel/IMU + verified revisit probe."""
import tempfile
import unittest
import json
from unittest.mock import patch
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from evaluation.offline_pose_graph import (se2, vector, relative, preserve_body, solve_graph,
    corroborate, icp, verify_revisit, require_finished, require_loops, JointMount, run, sha)


def edge(i,j,z,kind='odom'):
    return dict(i=i,j=j,z=z,kind=kind)


class PoseGraphTest(unittest.TestCase):
    def test_relative_convention_and_mount_preserves_height_tilt(self):
        a=se2([2,3,.7]);b=se2([-1,4,-.4])
        np.testing.assert_allclose(a@relative(a,b),b,atol=1e-12)
        body=a.copy();body[:3,:3]=Rotation.from_euler('ZYX',[.7,.2,-.1]).as_matrix();body[2,3]=.13
        mount=np.eye(4);mount[:3,:3]=Rotation.from_euler('z',.8).as_matrix();mount[:3,3]=[.12,.04,.3]
        changed=preserve_body(body,se2([4,-2,-.5]))
        np.testing.assert_allclose(np.linalg.inv(se2([4,-2,-.5]))@changed,np.linalg.inv(a)@body,atol=1e-12)
        self.assertAlmostEqual(changed[2,3],body[2,3]);np.testing.assert_allclose(np.linalg.inv(changed)@(changed@mount),mount,atol=1e-12)

    def test_square_loop_closes_with_first_pose_anchored(self):
        truth=[se2(v) for v in [[0,0,0],[1,0,0],[1,1,1.57],[0,1,3.14],[0,0,4.71],[1,0,6.28],[1,1,7.85]]]
        raw=[se2(vector(t)+np.array([.04*i,.02*i,.012*i])) for i,t in enumerate(truth)]
        edges=[edge(i,i+1,relative(raw[i],raw[i+1])) for i in range(len(raw)-1)]
        edges += [edge(0,4,relative(truth[0],truth[4]),'loop'),edge(1,5,relative(truth[1],truth[5]),'loop'),edge(2,6,relative(truth[2],truth[6]),'loop')]
        result,report=solve_graph(raw,edges)
        np.testing.assert_array_equal(result[0],raw[0])
        self.assertLess(np.linalg.norm(result[4][:2,3]-truth[4][:2,3]),.08)
        self.assertEqual(report['retained_loops'],3)

    def test_gross_false_loop_is_removed(self):
        raw=[se2([i*.2,0,0]) for i in range(8)]
        edges=[edge(i,i+1,relative(raw[i],raw[i+1])) for i in range(7)]
        edges += [edge(0,5,relative(raw[0],raw[5]),'loop'),edge(1,6,relative(raw[1],raw[6]),'loop'),edge(2,7,se2([4,2,1]),'loop')]
        result,report=solve_graph(raw,edges)
        self.assertEqual(report['rejected_loops'],1)
        self.assertLess(np.linalg.norm(result[-1][:2,3]-raw[-1][:2,3]),.02)

    def test_consistent_aliases_cannot_override_odometry(self):
        poses=[se2([i*.1,0,0]) for i in range(12)]
        aliases=[dict(edge(0,8,se2([0,0,0]),'loop'),support={0,1,8,9}),
                 dict(edge(3,11,se2([0,0,0]),'loop'),support={3,4,10,11})]
        # Repeated rooms can pass the short cycle test: it is not an identity proof.
        aliases=corroborate(aliases,poses,2*np.arange(12));self.assertEqual(len(aliases),2)
        edges=[edge(i,i+1,relative(poses[i],poses[i+1])) for i in range(11)]+aliases
        result,report=solve_graph(poses,edges)
        self.assertEqual(report['retained_loops'],0)
        self.assertLess(np.max([np.linalg.norm(error[:2]) for error in
            [vector(relative(a,b)) for a,b in zip(poses,result)]]),.001)
        with self.assertRaisesRegex(ValueError,'corroborated'):require_loops([])

    def test_final_validation_checks_resolved_loops_and_adjacent_strain(self):
        poses=[se2([i*.1,0,0]) for i in range(12)]
        edges=[edge(i,i+1,relative(poses[i],poses[i+1])) for i in range(11)]
        edges += [edge(0,8,relative(poses[0],poses[8]),'loop'),edge(3,11,se2([4,0,0]),'loop')]
        raw=np.array([vector(p) for p in poses])
        # First solve drops the gross loop; re-solve distributes strain but breaks retained loop.
        shifted=raw.copy();shifted[:,0]+=.015*np.arange(12)
        outputs=[SimpleNamespace(success=True,x=raw[1:].ravel(),cost=0.),
                 SimpleNamespace(success=True,x=shifted[1:].ravel(),cost=0.)]
        with patch('evaluation.offline_pose_graph.least_squares',side_effect=outputs):
            with self.assertRaisesRegex(ValueError,'retained loop'):solve_graph(poses,edges)
        strained=raw.copy();strained[1:,0]+=.07
        with patch('evaluation.offline_pose_graph.least_squares',return_value=SimpleNamespace(success=True,x=strained[1:].ravel(),cost=0.)):
            with self.assertRaisesRegex(ValueError,'adjacent odometry'):solve_graph(poses,edges[:-2])

    def test_cycle_requires_independent_windows_and_consistent_transform(self):
        poses=[se2([i*.1,0,0]) for i in range(12)]
        good=[dict(edge(0,8,relative(poses[0],poses[8]),'loop'),support={0,1,8,9}),dict(edge(3,11,relative(poses[3],poses[11]),'loop'),support={3,4,10,11})]
        self.assertEqual(len(corroborate(good,poses,2*np.arange(12))),2)
        bad=[dict(good[0]),dict(good[1],z=se2([1.4,0,0]))]
        self.assertEqual(corroborate(bad,poses,2*np.arange(12)),[])
        self.assertEqual(corroborate([good[0],dict(good[1],support={1,3,10,11})],poses,2*np.arange(12)),[])

    def test_disjoint_valid_windows_eight_seconds_apart_corroborate(self):
        poses=[se2([i*.1,0,0]) for i in range(12)]
        times=np.array([0,1,2,8,9,10,20,21,30,31,32,38])
        loops=[dict(edge(0,8,relative(poses[0],poses[8]),'loop'),support={0,1,8,9}),
               dict(edge(3,11,relative(poses[3],poses[11]),'loop'),support={3,4,10,11})]
        self.assertEqual(len(corroborate(loops,poses,times)),2)
        # Both endpoints must be independent in time; either close endpoint rejects.
        for first_gap,last_gap in ((4.99,8),(8,4.99)):
            close=times.astype(float);close[3]=first_gap;close[11]=30+last_gap
            self.assertEqual(corroborate(loops,poses,close),[])
        boundary=times.copy();boundary[3]=5;boundary[11]=35
        self.assertEqual(len(corroborate(loops,poses,boundary)),2)
        distant=times.copy();distant[3]=300;distant[11]=900
        self.assertEqual(len(corroborate(loops,poses,distant)),2)

    def test_single_wall_unobservable_and_corner_bidirectional(self):
        y,z=np.meshgrid(np.linspace(-1,1,30),np.linspace(.2,1.4,15));x=np.zeros(y.size)
        wall=np.column_stack([x,y.ravel(),z.ravel()]);wn=np.tile([1.,0,0],(len(wall),1))
        _,report=icp(wall,wn,wall,wn,np.eye(4));self.assertFalse(report['accepted'])
        other=np.column_stack([y.ravel(),x,z.ravel()]);on=np.tile([0.,1,0],(len(other),1))
        points=np.concatenate([wall,other]);normals=np.concatenate([wn,on]);transform=se2([.04,-.03,.025])
        source=(points-transform[:3,3])@transform[:3,:3];sn=normals@transform[:3,:3]
        fit=(points,normals);moving=(source,sn)
        measured,report=verify_revisit(fit,moving,fit,moving,np.eye(4))
        self.assertTrue(report['accepted'],report);np.testing.assert_allclose(measured,transform,atol=.002)
        # Fit can be observable while independent heldout observations show only one wall.
        wall_moving=((wall-transform[:3,3])@transform[:3,:3],wn@transform[:3,:3])
        wall_report=verify_revisit(fit,moving,(wall,wn),wall_moving,np.eye(4))[1]
        self.assertFalse(wall_report['accepted'])
        self.assertEqual(wall_report['heldout_forward']['reason'],'unobservable_heldout')
        far=(source+np.array([0,0,2]),sn)
        self.assertFalse(verify_revisit(fit,far,fit,far,np.eye(4))[1]['accepted'])
        # A plausible fit is insufficient when the independent heldout frames disagree.
        self.assertFalse(verify_revisit(fit,moving,fit,far,np.eye(4))[1]['accepted'])

    def test_mount_uses_only_joints_not_simulator_world_pose(self):
        model=Path(__file__).resolve().parents[1]/'simulation/scenarios/indoor_tiny_house.xml'
        mount=JointMount(model)
        sample=dict(lh=2.35,lk=.02,lw=1,rh=2.35,rk=.02,rw=2,hj=.6)
        a=mount(sample)
        b=mount(dict(sample,x=10000,y=-999,z=20,qw=0,qx=1,qy=0,qz=0))
        np.testing.assert_array_equal(a,b)
        changed=mount(dict(sample,hj=-.6))
        self.assertGreater(np.linalg.norm(a-changed),.1)

    def test_export_links_depth_exactly_and_refuses_without_verified_loops(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source';source.mkdir();(source/'keyframes').mkdir()
            for name in ('metadata.json','live_capture.jsonl','headless_result.json','scan_route.json'):
                (source/name).write_text('{}')
            frames=[];hashes={}
            for i in range(12):
                name=f'keyframes/{i:05d}.npz'
                np.savez_compressed(source/name,depth=np.full((24,32),1000,np.uint16),depth_source='deltar_tof_fusion')
                frames.append(dict(file=name,time_s=i));hashes[str(source/name)]=sha(source/name)
            sequence=dict(frames=frames,intrinsics=[20,20,15,11])
            poses=[se2([i*.1,0,0]) for i in range(12)]
            mounts=[np.eye(4) for _ in poses];packets=[dict(time_s=i,valid=1) for i in range(12)]
            loaded=(sequence,2*np.arange(12),poses,mounts,packets,hashes)
            loops=corroborate([dict(edge(0,8,relative(poses[0],poses[8]),'loop'),support=[0,1,8,9]),
                               dict(edge(3,11,relative(poses[3],poses[11]),'loop'),support=[3,4,10,11])],poses,2*np.arange(12))
            with patch('evaluation.offline_pose_graph.read_source',return_value=loaded),patch('evaluation.offline_pose_graph.find_loops',return_value=(loops,{})):
                output=root/'good';report=run(source,output)
            self.assertEqual(report['status'],'diagnostic_export_complete')
            for f in frames:
                self.assertTrue((output/f['file']).is_symlink())
                self.assertEqual(sha(source/f['file']),sha(output/f['file']))
            self.assertTrue((output/'map.ply').exists())
            saved=json.loads((output/'live_sequence.json').read_text())
            np.testing.assert_array_equal(saved['frames'][0]['camera_to_world'],poses[0])
            with patch('evaluation.offline_pose_graph.read_source',return_value=loaded),patch('evaluation.offline_pose_graph.find_loops',return_value=([],{})):
                rejected=root/'rejected'
                with self.assertRaisesRegex(ValueError,'corroborated'):run(source,rejected)
            self.assertFalse((rejected/'live_sequence.json').exists());self.assertFalse((rejected/'map.ply').exists())
            self.assertEqual(json.loads((rejected/'pose_graph_report.json').read_text())['status'],'rejected_no_export')

    def test_active_session_and_unconstrained_graph_refuse(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(ValueError,'finished'):require_finished(Path(d))
        with self.assertRaisesRegex(ValueError,'corroborated'):require_loops([])
        with self.assertRaisesRegex(ValueError,'corroborated'):require_loops([edge(0,4,np.eye(4),'loop')])

    def test_clean_shutdown_does_not_make_a_partial_route_exportable(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)
            (source/'STOP').touch()
            (source/'slam_shutdown.json').write_text('{"clean_stop":true}')
            (source/'scan_route.json').write_text('{"waypoints":[{},{},{}]}')
            (source/'headless_result.json').write_text(
                '{"success":true,"waypoints":3,"final_index":1}')
            with self.assertRaisesRegex(ValueError,'full-house route'):
                require_finished(source)

    def test_completed_full_route_is_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)
            (source/'STOP').touch()
            (source/'slam_shutdown.json').write_text('{"clean_stop":true}')
            (source/'scan_route.json').write_text('{"waypoints":[{},{},{}]}')
            (source/'headless_result.json').write_text(
                '{"success":true,"waypoints":3,"final_index":3}')
            require_finished(source)

    def test_keyframes_are_linked_without_duplicating_raw_measurements(self):
        import evaluation.offline_pose_graph as module
        self.assertTrue(hasattr(module,'link_keyframe'))
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'raw.npz';destination=Path(d)/'result.npz'
            source.write_bytes(b'immutable raw depth')
            original=source.read_bytes()
            module.link_keyframe(source,destination)
            self.assertTrue(destination.is_symlink())
            self.assertEqual(destination.read_bytes(),original)
            self.assertEqual(source.read_bytes(),original)

if __name__=='__main__':unittest.main()
