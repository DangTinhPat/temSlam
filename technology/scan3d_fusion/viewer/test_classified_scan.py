import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from classified_scan import load_scan, view_config


def product(folder):
    counts=dict(obstacle=2,uncertain=1,ground=0)
    files=dict(obstacle='obstacles.ply',uncertain='uncertain.ply',ground='ground.ply')
    for name,count in counts.items():
        points=np.arange(count*3,dtype='<f4').reshape(-1,3)
        header=('ply\nformat binary_little_endian 1.0\n'+f'element vertex {count}\n'
                'property float x\nproperty float y\nproperty float z\nend_header\n')
        (folder/files[name]).write_bytes(header.encode()+points.tobytes())
    report=dict(ground_source='gravity_ransac_labels_v3',pose_source='estimated final optimized poses',
                classification_pose_source='synchronized wheel/IMU + body_from_camera',map_quality_status='diagnostic/unvalidated',
                ground_truth_geometry=False,depth_modified=False,poses_modified=False,
                source_depth_sha256='a'*64,output_depth_sha256='a'*64,source_pose_sha256='b'*64,output_pose_sha256='b'*64,
                layer_points=counts,exported_files=files)
    (folder/'ground_report.json').write_text(json.dumps(report))
    return report


class ClassifiedScanTests(unittest.TestCase):
    def test_actual_layers_empty_layer_and_view_from_all_bounds(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);report=product(folder)
            layers,loaded=load_scan(folder)
            self.assertEqual(loaded,report)
            self.assertEqual(layers['ground'].shape,(0,3))
            np.testing.assert_array_equal(layers['obstacle'],np.arange(6).reshape(2,3))
            config=view_config(layers)
            manager=config['Visualization Manager'];displays=manager['Displays']
            self.assertEqual([d['Enabled'] for d in displays],[True,False,False,True])
            self.assertTrue(all(d['Topic']['Value'].startswith('/rb_classified/') for d in displays))
            self.assertEqual(manager['Views']['Current']['Focal Point'],dict(X=1.5,Y=2.5,Z=3.5))

    def test_wrong_report_counts_truncated_payload_and_nonfinite_points_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp);report=product(folder)
            report['layer_points']['obstacle']=3
            (folder/'ground_report.json').write_text(json.dumps(report))
            with self.assertRaises(ValueError):load_scan(folder)
            product(folder);path=folder/'obstacles.ply';data=path.read_bytes();path.write_bytes(data[:-1])
            with self.assertRaises(ValueError):load_scan(folder)
            product(folder);data=path.read_bytes();path.write_bytes(data[:-4]+np.array([np.nan],dtype='<f4').tobytes())
            with self.assertRaises(ValueError):load_scan(folder)

    def test_bad_provenance_or_wrong_ply_header_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)
            for changes in [dict(map_quality_status='accepted'),dict(ground_source='old'),dict(depth_modified=True),
                            dict(output_pose_sha256='c'*64),dict(exported_files=dict(obstacle='../obstacles.ply'))]:
                report=product(folder);report.update(changes)
                (folder/'ground_report.json').write_text(json.dumps(report))
                with self.assertRaises(ValueError):load_scan(folder)
            product(folder);path=folder/'obstacles.ply'
            path.write_bytes(path.read_bytes().replace(b'property float x',b'property uchar x'))
            with self.assertRaises(ValueError):load_scan(folder)


if __name__=='__main__':unittest.main()
