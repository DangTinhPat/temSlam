import unittest
import numpy as np
from evaluate_sofa_gap import surrounding_regions,ROOM_BOUNDS,RESOLUTION


class CoverageTests(unittest.TestCase):
    def test_quadrants_exclude_the_already_improved_gap(self):
        size=np.ceil((ROOM_BOUNDS[1]-ROOM_BOUNDS[0])/RESOLUTION).astype(int)
        before=np.zeros((size[1],size[0]),bool);after=before.copy()
        y,x=np.indices(before.shape)
        x=ROOM_BOUNDS[0,0]+(x+.5)*RESOLUTION;y=ROOM_BOUNDS[0,1]+(y+.5)*RESOLUTION
        after[(x>=-3)&(x<=-1.3)&(y>=-3.3)&(y<=-1.3)]=True
        regions=surrounding_regions(before,after,ROOM_BOUNDS)
        self.assertEqual(len(regions),4)
        self.assertTrue(all(r['added_supported_area_m2']==0 for r in regions.values()))
        regions=surrounding_regions(before,np.ones_like(before),ROOM_BOUNDS)
        self.assertTrue(all(r['candidate_fraction']==1 for r in regions.values()))


if __name__=='__main__':unittest.main()
