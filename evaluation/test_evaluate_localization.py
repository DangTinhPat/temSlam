import unittest
import numpy as np
from evaluate_localization import pose_errors, aggregate_errors


class EvaluationTests(unittest.TestCase):
    def test_fixed_alignment_and_wrapped_heading_error(self):
        # Simulator x=1 maps to map (10,1) after a 90 degree rotation.
        alignment=np.array([[0,-1,0,10],[1,0,0,0],[0,0,1,0],[0,0,0,1]],float)
        truth=np.eye(4); truth[0,3]=1
        estimate=alignment@truth;estimate[0,3]+=.03;estimate[1,3]+=.04
        error=pose_errors(estimate,truth,alignment)
        self.assertAlmostEqual(error['xy_m'],.05)
        self.assertAlmostEqual(error['yaw_deg'],0)
        a,b=np.deg2rad([179,-179])
        for pose,angle in ((estimate,a),(truth,b)):
            pose[:2,:2]=[[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]]
        self.assertAlmostEqual(abs(pose_errors(estimate,truth,np.eye(4))['yaw_deg']),2)

    def test_metrics_do_not_hide_unusable_frames(self):
        rows=[dict(xy_m=.03,yaw_deg=2,usable=True),dict(xy_m=.04,yaw_deg=-2,usable=False)]
        metrics=aggregate_errors(rows)
        self.assertAlmostEqual(metrics['all']['xy_rmse_m'],np.sqrt(.00125))
        self.assertEqual(metrics['usable']['samples'],1)
        self.assertAlmostEqual(metrics['usable']['xy_rmse_m'],.03)
        self.assertIsNone(aggregate_errors([])['all']['xy_rmse_m'])


if __name__=='__main__':unittest.main()
