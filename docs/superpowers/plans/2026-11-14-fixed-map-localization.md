# Fixed-map localization implementation plan

> Use superpowers:subagent-driven-development for the estimator and independent review; integrate and verify in the current authorized workspace. User approved the architecture in chat with “ok triển khai”.

**Goal:** Run planar-constrained RGB–ToF localization against the saved room map, from an explicit approximate body pose, with honest loss states and reproducible held-out evaluation.
**Spec:** technology/scan3d_fusion/navigation/README.md, milestone 1 and approved chat design.
**Architecture:** Build a fixed voxel reference from fusion keyframes and saved poses. Predict from wheel/IMU with the current camera mount; correct XY/yaw using gated point-to-plane registration. Provide replay and live-buffer CLI; do not issue motor commands.
**Tech stack:** Existing Python, NumPy, SciPy, DELTAR environment for sensor replay; no new dependencies or ROS requirement.

## Global constraints
- Never read simulator truth or saved mapping poses as localization measurements or initial pose.
- Caller explicitly supplies initial body x/y/yaw (radians in Python, degrees in CLI); z/roll/pitch retain level-floor odometry semantics.
- Reference is immutable. Outputs in new paths, never overwrite map inputs.
- Use only fusion depth with provenance; reject completed/corrected floor depth.
- A changing camera mount must not rotate the body estimate.
- Expose initializing/tracking/degraded/lost, correction acceptance/rejection reasons, overlap/residual/geometry/jump metrics, timestamps and pose usability.
- First delivery is localization, not navigation or global relocalization. Never issue motion commands.

## Task 1: Fixed reference and estimator
Files: navigation/fixed_map.py, navigation/test_fixed_map.py.
Interface: FixedReference(points, normals); FixedReference.from_session(Path); FixedMapLocalizer(reference, initial_pose=(x,y,yaw_rad)); update(timestamp_ns, packet, body_from_camera, depth_mm, intrinsics) -> JSON-safe dict including timestamp_ns, state, usable, world_from_body, world_from_camera, accepted, reason and matching metrics. world poses may be None before valid odometry.
- [ ] Write/run failing tests: noisy corner recovers pose, single wall/no overlap rejected, independent camera yaw preserves body, invalid/stale inputs, lost and recovery, reference remains unchanged.
- [ ] Implement using existing rigid/odom_body/vertical_points/planar_change; retain whole-system quality gates.
- [ ] Run tests and record evidence.

## Task 2: Replay/live runner
Files: navigation/run_localization.py, navigation/test_run_localization.py.
Inputs: replay session/localization_input.json with intrinsics and frames [{file,time_s,odometry,body_from_camera}]; each file is fusion-depth NPZ. Live reads existing session/rgbd.npz (t,K,mount,odometry,depth,depth_source).
CLI: --map PATH --initial-pose X Y YAW_DEG --output NEW_DIRECTORY plus --replay PATH or --live PATH.
- [ ] Test replay output, input provenance, explicit initialization, output collision, and stale live status.
- [ ] Implement append-only JSONL results, atomic latest status, run metadata, summary counts/latencies; wall-clock stale detection in live mode.
- [ ] Document commands and limitations; run replay through the real reference.

## Task 3: Held-out evidence and review
Files: evaluation/prepare_localization_replay.py, evaluation/test_prepare_localization_replay.py, evaluation/fixed_map_localization_results.md.
- [ ] Prepare measurements from a raw session different from map-source session: sensor render + existing DELTAR + ToF support, synchronized odometry and dynamic mount; no raw/map pose supplied to estimator.
- [ ] Record sensor source and timestamps, write separate truth only for offline evaluation.
- [ ] Run fixed-map replay, quantify tracking coverage/rejections/latency; compare pose errors only with an explicit independently justified map-to-simulator transform. Never align estimated trajectory to truth after the run and call it absolute accuracy.
- [ ] Independent review, resolve findings, run related regression suites and publish reproducible results. Any inability to validate tracking must be reported explicitly.

## Evidence-driven correction (14 November)

Initial independent replay:200 frames,129 usable, XYRMSE0.285m versus0.014m for
wheel/IMU from the same explicit initial pose. Small per-frame surface residual
was not sufficient: corrections drifted during a neck sweep. Therefore Task1
must include a short bounded odometry-consistent multiview window, freshness
checks for each current view and consistent hypotheses before commitment.
This is a correction to acceptance quality within milestone1, not permission
to publish the initial result as validated navigation. Keep original results
under localization-baseline-20261114 for a visible before/after comparison.

## Delivery checkpoint

Tasks1–3 implementation, independent review and measurements completed.38 relevant
tests pass. Added multiview checks and fixed live buffer age after review.
Milestone1 empirical acceptance is NOT complete: XYRMSE27.2/54.2cm on disjoint
60s query intervals, worse than odometry1.4/2.9cm. No motion integration.
Full evidence and limitations: evaluation/fixed_map_localization_results.md.
Current feature branch retained; pre-existing workspace edits preserved.
