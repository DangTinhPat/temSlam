import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import h5py
import numpy as np
from PIL import Image
import replay_fusion_only as replay


class ReplayTests(unittest.TestCase):
    def test_cached_predictions_do_not_render_or_rerun_model_or_change_poses(self):
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source';out=Path(tmp)/'out'
            (source/'inputs').mkdir(parents=True);(source/'prediction').mkdir()
            (source/'inputs/data.json').write_text('{}')
            (source/'source.json').write_text(json.dumps(dict(max_sensor_timestamp_offset_s=0)))
            sequence=dict(frames=[dict(file='keyframes/00000.npz',camera_to_world=np.eye(4).tolist())],
                intrinsics=[500,500,320,240],bounds_m=[[-1,-1,0],[1,1,2]])
            (source/'live_sequence.json').write_text(json.dumps(sequence))
            Image.fromarray(np.full((480,640),1000,np.uint16)).save(source/'prediction/00000_depth.png')
            with h5py.File(source/'inputs/00000.h5','w') as h:
                for name,value in dict(hist_data=np.tile([1.,.01],(64,1)),fr=np.zeros((64,4)),mask=np.ones(64,bool)).items():
                    h.create_dataset(name,data=value)
            mesh=SimpleNamespace(export=lambda p:Path(p).write_text('mesh'),bounds=np.array([[-1,-1,0],[1,1,2]]))
            with patch('sys.argv',['replay',str(source),str(out)]), \
                    patch.object(replay,'Sensors',side_effect=AssertionError('must reuse sensor inputs')), \
                    patch.object(replay.subprocess,'run',side_effect=AssertionError('must reuse model outputs')), \
                    patch.object(replay,'tof_support',side_effect=lambda depth,*args:depth), \
                    patch.object(replay,'build_surface',return_value=(mesh,.04)):
                replay.main()
            result=json.loads((out/'live_sequence.json').read_text())
            self.assertEqual(result['frames'],sequence['frames'])
            self.assertEqual(result['bounds_m'],sequence['bounds_m'])
            self.assertTrue(json.loads((out/'source.json').read_text())['model_predictions_reused'])
            with np.load(out/'keyframes/00000.npz',allow_pickle=False) as data:
                np.testing.assert_array_equal(data['depth'],1000)
                self.assertNotIn('floor_plane',data.files)


if __name__=='__main__':unittest.main()
