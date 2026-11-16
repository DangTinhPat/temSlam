import unittest
import time
import io
from types import SimpleNamespace
import numpy as np
from app import Viewport, demo_grid, NavigationApp


class AppTests(unittest.TestCase):
    def test_viewport_roundtrip_and_north_up(self):
        grid=demo_grid();view=Viewport(grid,800,600)
        for xy in ([.5,.5],[2.4,3.2],[4,4]):
            np.testing.assert_allclose(view.world(view.screen(xy)),xy,atol=1e-10)
        self.assertLess(view.screen([1,2])[1],view.screen([1,1])[1])

    def test_click_preview_start_cancel_and_obstacle_on_tk_canvas(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw()
        try:
            app=NavigationApp(root,demo_grid(),(.8,.8,0),'demo')
            app.select_goal([4.2,3.2])
            self.assertGreater(len(app.preview),2)
            app.begin()
            self.assertEqual(app.navigator.state,'planning')
            app.cancel()
            self.assertEqual(app.navigator.state,'cancelled')
            app.select_goal([2.5,1.5])
            self.assertFalse(app.preview)
            self.assertEqual(app.navigator.state,'cancelled')
        finally:root.destroy()

    def test_native_explicit_new_goal_rearms_watchdog_and_terminal_reason_stays(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw()
        try:
            app=NavigationApp(root,demo_grid(),(.8,.8,0),'native')
            runtime=SimpleNamespace(backend=None,ready=False,watchdog_lost=True,message='RB ready')
            def rearm():runtime.watchdog_lost=False
            runtime.rearm=rearm;runtime.observe=lambda now:(None,None)
            app.runtime=runtime
            app.select_goal([4.2,3.2])
            self.assertFalse(runtime.watchdog_lost)
            app.navigator.state='lost';app.message='Mất pose đã dừng';app.running=False
            app.tick()
            self.assertEqual(app.message,'Mất pose đã dừng')
        finally:root.destroy()

    def test_observation_processing_time_counts_toward_pose_expiry(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw()
        try:
            app=NavigationApp(root,demo_grid(),(.8,.8,0),'native')
            def observe(now):
                time.sleep(.12)
                return (dict(xy=[.8,.8],yaw=0.,stamp=now-.25,usable=True,uncertainty_m=0.),
                        dict(clear=True,stamp=now,reason='test'))
            app.runtime=SimpleNamespace(observe=observe,command=lambda result:None,ready=True)
            app.log=io.StringIO();app.navigator.set_goal([.8,.8],[4.2,3.2]);app.running=True
            app.tick()
            self.assertEqual(app.navigator.state,'lost')
            self.assertFalse(app.running)
        finally:root.destroy()


class MeshNativeBoundaryTests(unittest.TestCase):
    def test_mesh_projection_cannot_start_native_motion(self):
        import tkinter as tk
        from unittest.mock import patch
        root=tk.Tk();root.withdraw()
        try:
            grid=demo_grid();grid.demo_only=True
            app=NavigationApp(root,grid,(.8,.8,0),'native')
            with patch('runtime.NativeNavigation',side_effect=AssertionError('Native startup forbidden')):
                app.start_native()
            self.assertIsNone(app.runtime)
            self.assertIn('chỉ dùng demo',app.message)
        finally:root.destroy()


if __name__=='__main__':unittest.main()
