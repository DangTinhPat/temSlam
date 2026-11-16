import tempfile
import unittest
from pathlib import Path
import numpy as np
from mesh_grid import build_mesh_grid
from grid import FREE, OCCUPIED, UNKNOWN, NavigationGrid


def rectangle(x0, y0, x1, y1, z=0.):
    return [[[x0,y0,z],[x1,y0,z],[x1,y1,z]],
            [[x0,y0,z],[x1,y1,z],[x0,y1,z]]]


class MeshGridTests(unittest.TestCase):
    def test_floor_projection_supports_route_and_preserves_provenance(self):
        grid=build_mesh_grid(np.array(rectangle(-2,-2,2,2)))
        self.assertTrue(grid.demo_only)
        self.assertEqual(grid.cells[grid.world_to_cell([0,0])],FREE)
        self.assertTrue(grid.plan([-.8,0],[.8,0]))
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'mesh.npz';grid.save(path)
            self.assertTrue(NavigationGrid.load(path).demo_only)

    def test_unobserved_hole_is_not_filled(self):
        triangles=rectangle(-2,-2,-.3,2)+rectangle(.3,-2,2,2)
        grid=build_mesh_grid(np.array(triangles))
        self.assertEqual(grid.cells[grid.world_to_cell([0,0])],UNKNOWN)
        with self.assertRaises(ValueError):grid.plan([-1,0],[1,0])

    def test_vertical_wall_blocks_even_without_projected_area(self):
        triangles=rectangle(-2,-2,2,2)+[[[0,-2,0],[0,2,0],[0,2,.5]],[[0,-2,0],[0,2,.5],[0,-2,.5]]]
        grid=build_mesh_grid(np.array(triangles))
        self.assertEqual(grid.cells[grid.world_to_cell([.001,0])],OCCUPIED)
        with self.assertRaises(ValueError):grid.plan([-1,0],[1,0])

    def test_low_obstacle_and_overhang_survive_floor_projection(self):
        for z in [.04,.35]:
            grid=build_mesh_grid(np.array(rectangle(-2,-2,2,2)+rectangle(-.2,-.2,.2,.2,z)))
            self.assertEqual(grid.cells[grid.world_to_cell([0,0])],OCCUPIED)
            self.assertFalse(grid.segment_clear([-.8,0],[.8,0]))

    def test_steep_low_surface_not_classified_as_floor(self):
        triangles=np.array(rectangle(-2,-2,2,2))
        triangles[:,:,2]=triangles[:,:,0]*.5
        with self.assertRaisesRegex(ValueError,'floor'):build_mesh_grid(triangles)

    def test_invalid_geometry_is_rejected(self):
        for triangles in [[],np.zeros((1,2,3)),np.full((1,3,3),np.nan)]:
            with self.assertRaises(ValueError):build_mesh_grid(np.array(triangles))


if __name__=='__main__':unittest.main()
