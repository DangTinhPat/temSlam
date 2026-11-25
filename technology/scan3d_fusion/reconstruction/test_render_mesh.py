"""Rendering a house-sized mesh must not hit MuJoCo's STL face limit."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from PIL import Image
import trimesh

from render_mesh import main


class RenderMeshTest(unittest.TestCase):
    def test_preview_preserves_mesh_above_stl_face_limit(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / 'large.stl'
            output = Path(tmp) / 'preview.png'
            box = trimesh.creation.box()
            box.faces = np.tile(box.faces, (16667, 1))  # 200,004 faces, eight unique vertices
            box.export(source)
            before = source.read_bytes()
            with patch.object(sys, 'argv', ['render_mesh.py', str(source), '--output', str(output)]):
                main()
            self.assertEqual(source.read_bytes(), before)
            with Image.open(output) as preview:
                self.assertEqual(preview.size, (1600, 840))


if __name__ == '__main__':
    unittest.main()
