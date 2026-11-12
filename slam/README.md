# SLAM

Vision, ToF mapping, DELTAR, calibration, mesh reconstruction and RViz tooling are
organized in [technology/scan3d_fusion](../technology/scan3d_fusion/README.md).

This workspace currently contains two independent paths:

- ToF-only native occupancy mapping from encoder/gyro odometry and 8×8 range data.
- RGB + ToF depth completion and TSDF mesh reconstruction using known simulated poses.

The second path is not a complete camera tracking or SLAM implementation. The simulator,
RB model, GUI and generated datasets remain in their workspace-level directories.
