# Drive release regression — 2026-11-10

The simulator overrode the controller's drive profile from 0.3 m/s² and
1 m/s³ to 0.9 m/s² and 10 m/s³. The preview wheel command includes a jerk
term; removing these overrides reduces the abrupt pitch motion on STOP.
Firmware defaults and the GUI STOP command are unchanged.

Reproduce from the repository root:

```sh
python evaluation/drive_stop.py --runner /tmp/temslam-pc-check/bin/rb1_sim
```

MuJoCo flat floor, 3 kHz, home stand-up, height command at 16 s, drive at
25 s, STOP at 29 s. IMU model enabled, unit 648 unless indicated. Peak
pitch rate is measured during the five seconds after STOP.

| Profile / scenario | Peak pitch rate (rad/s) | Final state |
|---|---:|---|
| Old overrides, +0.3 m/s, height 0.14 m | 0.4900 | Balancing |
| Defaults, +0.3 m/s, height 0.14 m | 0.1165 | Balancing |
| Defaults, −0.3 m/s | 0.1204 | Balancing |
| Defaults, +0.6 m/s | 0.1343 | Balancing |
| Defaults, +0.3 m/s, height 0.20 m | 0.0951 | Balancing |
| Defaults, +0.3 m/s, IMU unit 208 | 0.1184 | Balancing |

All runs finish without a fault. At +0.3 m/s and height 0.14 m, peak pitch
rate falls 76%. Body travel after STOP increases from 0.20 m to 0.32 m.
The softer profile improves abruptness, but does not eliminate positional
rebound: at 34 s the body is about 0.004 m behind its furthest position in
the default test, 0.095 m at +0.6 m/s, and 0.058 m at height 0.20 m.
These are body-position measurements, not axle odometry. Hardware, slopes
and obstacle stopping clearance have not been validated.
