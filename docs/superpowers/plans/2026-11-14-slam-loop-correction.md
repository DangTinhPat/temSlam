# Restore optimized cuVSLAM poses as the map geometry authority

User priority: a consistent scanned map before navigation. Keep wheel/IMU as
continuous local odometry and an initial gravity/frame anchor; it must not veto
or replace globally optimized cuVSLAM poses used to reconstruct the map.

Observed: only 4/483 original mapping frames accepted visual correction; 2715
extra views propagated from odometry anchors. Existing map input gaps reach
21.66 seconds; raw capture is continuous at 15 Hz. Surface matching is sequential,
not a global pose graph. All original data stays unchanged.

- [x] Run a separate cuVSLAM-only replay on cached RGB/fusion depth; record
  tracking coverage, loop evidence and optimized versus immediate poses.
- [x] Reproduce suppression of a large/global historical correction in tests.
- [x] Use a single initial wheel/IMU gravity alignment for the entire optimized
  trajectory in live mapping and offline replay; retain cuVSLAM relative poses.
- [x] Stop sequential surface matching from overriding optimized SLAM in the
  main finalization path. Reintegrate depth after all historical pose updates.
- [x] If tracking evidence demands it, replay continuous raw sensor observations
  rather than adding odometry-only views. Do not change timestamps or use truth
  depth/pose in the estimator.
- [ ] Independently score trajectory and map consistency, review, rebuild and
  open the best verified result. Keep navigation disabled if acceptance fails.

Trajectory scoring and read-only review completed; 119 tests pass. Map acceptance
remains unmet: sparse replay ATE142.31m, continuous5Hz two runs ATE76.27m/9.24m.
No bad result promoted or mesh rebuilt as a successful navigation map. See
evaluation/slam_loop_results.md for measured failure and reproducible commands.
