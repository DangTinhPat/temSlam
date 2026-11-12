import unittest
import numpy as np
from complete_floor import fill_supported_gaps, grid_mesh, blocked_cells, confirmed_drops

class FloorCompletionTests(unittest.TestCase):
    def test_conflicting_evidence_does_not_permanently_block_floor(self):
        floor=np.zeros((20,20),np.uint16);obstacles=np.zeros_like(floor);drops=np.zeros_like(floor)
        floor[4,4]=12;obstacles[4,4]=2
        floor[10,10]=4;obstacles[10,10]=5
        floor[15,15]=30;drops[15,15]=2
        blocked=blocked_cells(floor,obstacles,drops)
        self.assertFalse(blocked[4,4])
        self.assertTrue(blocked[10,10]);self.assertTrue(blocked[10,11])
        self.assertTrue(blocked[15,15])
        self.assertFalse(blocked[0,0])

    def test_enclosed_gap_and_obstacle_and_outside(self):
        observed=np.zeros((80,80),bool);observed[10:70,10:70]=True
        observed[30:38,30:38]=False
        blocked=np.zeros_like(observed);blocked[45:51,45:51]=True
        known,filled=fill_supported_gaps(observed,blocked)
        self.assertTrue(filled[30:38,30:38].all())
        self.assertFalse(filled[blocked].any());self.assertFalse(known[blocked].any())
        self.assertFalse(filled[:10].any());self.assertFalse(filled[:,:10].any())
        mesh=grid_mesh(filled,[0,0],.05,np.array([0,0,1,-.02]))
        np.testing.assert_allclose(mesh.vertices[:,2],.02)

    def test_large_enclosed_gap_is_filled_but_concave_exit_stays_open(self):
        observed=np.zeros((100,100),bool);observed[10:90,10:90]=True
        observed[25:75,25:75]=False
        known,filled=fill_supported_gaps(observed,np.zeros_like(observed))
        self.assertTrue((known|filled)[10:90,10:90].all())
        observed[10:50,45:60]=False
        _,filled=fill_supported_gaps(observed,np.zeros_like(observed))
        self.assertFalse(filled[50,50]);self.assertFalse(filled[15,50])
        _,empty=fill_supported_gaps(np.zeros((20,20),bool),np.zeros((20,20),bool))
        self.assertFalse(empty.any())

    def test_repeated_drop_is_preserved_without_punching_noise_holes(self):
        floor=np.zeros((80,80),np.uint16);floor[10:70,10:70]=10
        floor[35:40,35:40]=0
        drops=np.zeros_like(floor);drops[35:40,35:40]=4
        drops[20,20]=2;drops[25,25]=4
        blocked=confirmed_drops(floor,drops)
        self.assertTrue(blocked[35:40,35:40].all())
        self.assertFalse(blocked[20,20]);self.assertFalse(blocked[25,25])
        known,filled=fill_supported_gaps(floor>=2,blocked)
        self.assertFalse((known|filled)[blocked].any())

    def test_floor_mesh_has_one_boundary_and_no_interior_holes(self):
        observed=np.zeros((80,80),bool);observed[10:70,10:70]=True
        observed[20:60,20:60]=False
        known,filled=fill_supported_gaps(observed,np.zeros_like(observed))
        plane=np.array([.01,-.02,1.,-.02])
        mesh=grid_mesh(known|filled,[0,0],.05,plane)
        np.testing.assert_allclose(mesh.vertices@plane[:3]+plane[3],0,atol=1e-12)
        self.assertEqual(mesh.euler_number,1)
        self.assertFalse((known&filled).any())

    def test_invalid_grid_rejected(self):
        with self.assertRaises(ValueError):
            fill_supported_gaps(np.zeros((2,2),bool),np.zeros((3,3),bool))
        with self.assertRaises(ValueError):
            fill_supported_gaps(np.zeros((2,2),bool),np.zeros((2,2),bool),0)

if __name__=='__main__':unittest.main()
