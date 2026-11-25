"""Sensor transport must preserve odometry when its consumer is slow."""
import io
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from backend import NativeBackend


class BackendTransportTest(unittest.TestCase):
    def test_map_viewer_does_not_inherit_pythonpath_from_another_python(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binaries = root / 'firmware/build/bin'
            binaries.mkdir(parents=True)
            runner = binaries / 'rb1_sim'
            runner.touch()
            runner.chmod(0o755)
            (binaries / 'rb_tof_mapper').touch()
            viewer = root / 'technology/scan3d_fusion/viewer/tof_map_viewer.py'
            viewer.parent.mkdir(parents=True)
            viewer.touch()
            backend = NativeBackend(root)
            backend.process = SimpleNamespace(args=['rb1_sim'])
            with patch.dict(os.environ, {'PYTHONPATH': '/other/python/site-packages'}), \
                    patch('backend.subprocess.Popen') as popen:
                backend._open_map_view()
            self.assertIn('env', popen.call_args.kwargs)
            self.assertNotIn('PYTHONPATH', popen.call_args.kwargs['env'])

    def test_slow_mapper_receives_every_sensor_packet_in_order(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binaries = root / 'firmware/build/bin'
            binaries.mkdir(parents=True)
            runner = binaries / 'rb1_sim'
            runner.touch()
            runner.chmod(0o755)
            (binaries / 'rb_tof_mapper').touch()
            viewer = root / 'technology/scan3d_fusion/viewer/tof_map_viewer.py'
            viewer.parent.mkdir(parents=True)
            viewer.write_text(
                'import pathlib,sys,time\n'
                'time.sleep(.15)\n'
                'session=pathlib.Path(sys.argv[sys.argv.index("--session")+1])\n'
                '(session/"received.log").write_text(sys.stdin.read())\n')
            packets = ''.join(f'TOFMAP {i} ' + '0 ' * 76 + '\n' for i in range(1000))
            backend = NativeBackend(root)
            backend.process = SimpleNamespace(args=[], stdout=io.StringIO(packets))
            backend._open_map_view()
            try:
                backend._read_output()
            finally:
                backend._close_map_view()
            self.assertEqual((backend.map_session / 'received.log').read_text(), packets)
            self.assertEqual(backend._map_dropped, 0)


if __name__ == '__main__':
    unittest.main()
