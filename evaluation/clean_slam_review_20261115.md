# Independent clean SLAM acceptance review — 2026-11-15

Reviewer scope: read-only production review and small CPU input checks. No GPU tracker or full test suite launched. This document is the only owned artifact. Existing dirty changes were preserved.

## Current decision

No navigation-quality replacement map is accepted. The recorded 750-frame runs have first-pose ATE of about 10.84 m and 135.17 m, despite successful SLAM metrics. The guarded run stops at frame 73 because local camera rotation is lost. A complete API trajectory is not evidence of complete tracking.

Existing fixes correctly retain one initial wheel/IMU alignment, unrestricted optimized pose history, camera neck lever arm, a local-motion check separate from loop correction, and the last accepted history before an erroneous SDK update. They do not establish tracking quality or floor correctness.

## Measured input failure, independently reproduced on CPU

Prepared source: `datasets/processed/scan3d_fusion/room-continuous-probe-20261114`.

All 750 saved `keyframes/*.npz` depths were inspected. **63/750 contain zero usable depth pixels**. Zero-depth runs (inclusive, zero-based) are 370–380, 492–509, 530–537, 664–681, 701–708. Longest run is 18 frames, spanning 3.4 s between first and last timestamps, with 3.6 s until the next frame.

Depth coverage quantiles [min, 5%, 10%, median, 90%, max] are [0, 0, .27688, .52604, .74407, .76889]. Aggregate valid-pixel totals in `source.json` therefore conceal substantial contiguous RGB-D failure.

This is not simply absent ToF returns:

| Frame | Valid ToF zones | ToF means min/median/max (m) | Raw DELTAR min/median/max (m) | Supported depth |
|---|---:|---|---|---|
| 370 | 64 | .455/.467/.480 | .640/.669/2.015 | all zero |
| 500 | 64 | .278/.286/.294 | .694/.729/1.806 | all zero |
| 530 | 64 | .287/.337/.409 | .634/.684/1.923 | all zero |

Concrete boundary: `reconstruction/depth_support.py:108` uses nearby predictions within max(.2 m, 3 sigma) to derive bias, `:112` caps bias at .15 m, and `:119–120` zero unsupported predictions. When the near scene is outside the pretrained model's useful prediction range, valid close ToF measurements cannot rescue dense predictions. Keeping these pixels unknown is correct; accepting the resulting sequence as observable RGB-D tracking is not.

`evaluation/prepare_localization_replay.py:112` checks only whether the whole session contains any valid pixels. `live/workers.py:40–45` validates provenance but does not establish per-frame observability. `RgbdSlam.track` accepts all-zero uint16 depth (`mapping/cuvslam/track_rgbd.py:102–117`) as required by the broad SDK image contract. A pose return is then trusted until the wheel/IMU motion residual rejects it.

The existing diagnostic still returns all 750 poses. Observation counts during the first all-zero run decay from 200 to 8; later zero-depth runs reach 0–3 observations. This explains a concrete source of late false success and catastrophic extrapolation. It does **not** explain the first guarded failure at index 73: that frame has approximately 71.9% supported pixels, all 64 ToF zones valid, and about 269/381 Shi–Tomasi RGB corners supported by depth. Initial angular tracking loss and later depth starvation must be investigated separately.

Small reproducible checks (no simulator truth or estimator modifications):

```python
from pathlib import Path
import json, numpy as np, h5py
from PIL import Image
p = Path('datasets/processed/scan3d_fusion/room-continuous-probe-20261114')
frames = json.loads((p/'localization_input.json').read_text())['frames']
coverage = []
for f in frames:
    with np.load(p/f['file'], allow_pickle=False) as sample:
        coverage.append(np.count_nonzero(sample['depth']) / sample['depth'].size)
print(np.flatnonzero(np.asarray(coverage) == 0))
for i in (370, 500, 530):
    with h5py.File(p/'inputs'/f'{i:05d}.h5') as h:
        hist, valid = h['hist_data'][:], h['mask'][:].astype(bool)
    prediction = np.asarray(Image.open(p/'prediction'/f'{i:05d}_depth.png'))/1000
    print(i, valid.sum(), np.quantile(hist[valid, 0], [0,.5,1]),
          np.quantile(prediction, [0,.5,1]))
```

## Input and SDK contracts checked

Local import through `mapping/cuvslam/run_python.sh` reports **17.0.1+cu12**. No tracker was constructed for these checks. Installed defaults: `use_motion_model=True`, `max_frame_delta_s=1.0`, `RGBDSettings.enable_depth_stereo_tracking=False`, pinhole camera, identity camera-to-rig transform. The 5 Hz input does not exceed the SDK max-gap default. RGBD depth-stereo default is already false; recommending toggling it off would be a no-op.

`live/sensors.py:33–34` derives the 640×480 pinhole focal length from the actual renderer FOV and principal point (319.5,239.5); prepared K is [579.41125497,579.41125497,319.5,239.5]. `:49–51` derives body-to-camera from each synchronized head pose and converts OpenGL to OpenCV. Camera and ToF render after the same `mj_forward` call. The adapter supplies contiguous uint8 RGB and uint16 millimeter depth with scale 1000, which matches the installed contract. Current evidence does not support an RGB/BGR, meters/mm, or static-neck-extrinsic root cause.

A minor projection convention difference exists: `capture/capture_mujoco.py:102–103` uses width/2,height/2 for projected zone rectangles while camera K uses half-minus-.5. A half-pixel rectangle shift is too small to explain 9.4° angular tracking failure or meter-scale drift. The exact per-point consistency mask already uses K; do not prioritize this as the catastrophic cause.

The DELTAR loader has a fixed NYU focal metadata value (`model/deltar/src/dataloader/zjuL5.py:60–65`), but the current model invocation only receives RGB and additional ToF inputs (`inference/run_demo.py:77–85`), not that focal. Changing this unused metadata would not fix tracking. The learned RGB geometry remains a domain/calibration limitation, to measure rather than conceal.

## Controlled next experiments

1. Use the genuine native 15 Hz, fixed-neck, textured sequence proposed by the coordinator. Preserve sensor timestamps and all planned frames. Report depth coverage, ToF-zone count, and observations at every frame. This tests whether the basic measured RGB-D contract works without simultaneous neck acceleration, blank walls, and severe depth starvation.
2. On identical saved inputs, change only SDK motion-model use. Its default is true; a neck acceleration/reversal can invalidate constant-motion image prediction. Compare the first failure index and local angular residual, not just total pose count. This is a hypothesis, not a confirmed cause. Do not combine it with depth or frame-rate changes.
3. A true static positive control should repeat an actually captured RGB-D pair at increasing native timestamp intervals and check near-zero output motion. Distinguish it from the moving positive control: a constant-pose estimator must fail the moving sequence.
4. Before a long replay, reject or explicitly flag all-zero-depth periods as an input-quality failure. Do not fabricate timestamps, fill depths, replace poses, skip the hard segment, or weaken the consistency mask simply to obtain completion. Minimum feature/coverage thresholds require empirical calibration; a single arbitrary threshold is not a map-quality certificate.

## Acceptance for the new short probe

Primary reference is **full SE3 first-pose anchoring once**: `truth[0] @ inv(estimate[0]) @ estimate`, already implemented by `mapping/cuvslam/evaluate.py:26`. Simulator truth is evaluator-only; this alignment is not fed into measurements, initialization, or exported maps. Post-fit trajectory SE3 remains secondary diagnostic information; fit scale is prohibited.

Record all of the following before interpreting quality:

- Planned/tracked count, failure reason/index, monotonic native timestamps, actual interval statistics, and finite rigid poses.
- First-pose anchored translation RMSE, maximum and endpoint error; absolute orientation RMSE, maximum and endpoint error; adjacent translation/rotation RPE and path lengths. Existing metrics omit accumulated absolute orientation error. A short nearly stationary path can have small position ATE while its map is badly rotated.
- Last optimized history and local odometry separately; every accepted timestamp must exist in final history. Any global optimization must preserve the complete coherent final graph.
- Geometry tolerance tied to the intended map resolution and observation range. At 4 m, 1° angular error displaces points about 7 cm, already exceeding a 4 cm voxel. Passing a loose translation metric alone cannot certify matching surfaces.
- Repeatability across at least two identical GPU replays because prior measurements vary greatly. Report every run rather than selecting the best.

A 150-frame / 10-second probe can establish basic input/tracking behavior. It cannot establish successful loop closure unless the camera actually revisits overlapping geometry. Loop-closure acceptance additionally requires measured revisit consistency before/after graph optimization, first-pose anchored map geometry, and no duplicate shifted wall/furniture surfaces. Metrics flags alone never meet that criterion.

## Floor-free export semantics

`GroundSegmenter` classifies measured pixels using synchronized local wheel/IMU pose and mount; it preserves depth and requires repeated measured plane evidence. The resulting camera-local pixel class and camera-local plane do not become intrinsically invalid merely because optimized world poses change. A physical floor measurement may remain a physical floor measurement even when its optimized world placement drifts.

The potential conflict is world consistency: `live/workers.py:121–126` measures labels in local odometry and `:137–153` places the same labeled points using unrestricted optimized camera poses. The transformed floor planes can therefore disagree or cease to sit at world z=0. Do not erase this evidence by flattening each pose or reclassifying against an incorrect global z prior. `reconstruction/segment_scan.py:51` classifies using saved optimized poses, while `ground_segmentation.py:49–50,66,84,109` requires calibrated floor near z=0; this offline path has a stronger world-frame prerequisite than local labeling.

Safe floor-free representation:

- Preserve original depth bytes, timestamps, labels, local plane/provenance, and coherent final optimized poses.
- Hide only confidently classified `GROUND` pixels for an obstacle display/export. Retain uncertain measured geometry as a distinct layer; do not label it free. Invalid/no-return depth remains unknown.
- Low obstacles survive regardless of small global z. Test 3 cm objects, raised platforms, slopes, plane boundaries, missing depth, and shifted/tilted optimized poses. Sub-centimeter geometry remains below the current documented noise/classification limit.
- An obstacle-only cloud or mesh does not certify traversable volume. Navigation must keep its independent measured floor/ray evidence and unknown blocking behavior (`navigation/grid.py:38`), not infer free space from omitted floor/unknown points.
- Source timestamps/frame conventions are required; recompute placement from one complete optimized trajectory after LC. If the trajectory is wrong, export can remain diagnostic but cannot become an accepted map by floor removal.

No generic global height cutoff or deletion by world z is acceptable on a drifted pose graph. Existing strict navigation additionally assumes a flat near-z0 floor and a limited vertical envelope (`navigation/grid.py:161–162,194–203`); this is a documented prototype, not a general 3D occupancy map.

## Review status

Awaiting the implementation diff and genuine fixed-neck probe results. No production edits, dependency changes, commits, GPU replay, or broad test reruns performed. No claim that the physical robot or full room loop is validated.

## Focused review package 1 — evaluator and guarded diagnostic

What this change does: Adds isolated prepared-truth evaluation, first-pose orientation/endpoint errors and one-second relative errors. The diagnostic can stop on the local wheel/IMU check while preserving the prior accepted optimized graph, and never saves an SDK database.

Review snapshot SHA-256:

- `evaluation/replay_slam_probe.py`: `01134d7a4a30bb3722aa215e0cf6e1cc38174419d6666923455093e64949ccf0`
- `evaluation/evaluate_slam_map.py`: `12584266250755c1cecc30849448902825f794d538e6c52ce48297703d90c386`
- `evaluation/test_slam_probe.py`: `1d0772b1451af21a095a1c172dcb6344f882dc18a370ff0823370e4cb8989390`

**Must fix 1: Failed guarded probes exit successfully** (`evaluation/replay_slam_probe.py:86`, `:89`). `run()` correctly returns `complete=False`, but `main()` discards it and the entrypoint exits normally. A shell command or `subprocess.run(..., check=True)` therefore treats a rejected frame as successful. Match production replay semantics: return 0 for complete, 2 for incomplete, then `sys.exit(main())`; add an exit regression including first-frame rejection. Identified by tracing the original `main()` and entrypoint, then sent directly to coding lead and coordinator. The coding lead reports a red regression (`None != 2`) before the fix; independent post-fix invocation returns 2.

The exact prepared file/time matching and full body-plus-mount truth transform read correctly. Truth remains isolated from replay. The full first-pose SE3 anchor and orientation error formula are correct; one-second pairs use native timestamps and explicit tolerance. Accepted optimized history is copied after sanity acceptance, so a subsequent rejected SDK mutation cannot alter it. Empty and one-frame evaluator paths return absent relative/SE3 metrics rather than invented values. Whole-map wheel/IMU anchoring is still the separately authorized next change.

Focused command: `technology/scan3d_fusion/mapping/cuvslam/run_python.sh -m unittest discover -s evaluation -p test_slam_probe.py -v` — **4 tests passed** in 0.046 s. This is CPU/fake-tracker verification only.

Verdict: fix 1 before treating guarded CLI status as a test gate. No further blocker found in the reviewed evaluator/snapshot behavior.

Not checked: GPU tracking quality, new fixed-neck measurements, full room loop, hardware, or a broad test suite. Existing API/default and depth measurements above remain diagnostic evidence, not product acceptance.


## Focused review package 2 — failure status, zero depth, and guarded anchoring

The coding lead fixed finding 1. Current `replay_slam_probe.main` returns 2 for incomplete reports and its entrypoint calls `sys.exit(main())`. Independent post-fix invocation with an incomplete mocked report returned 2.

Current SHA-256:

- `evaluation/replay_slam_probe.py`: `cd03be7d19b742b1097c6e13a574ab55a95b1df6a12a429cc7f5a8cb95e7ddb9`
- `evaluation/evaluate_slam_map.py`: `12584266250755c1cecc30849448902825f794d538e6c52ce48297703d90c386`
- `evaluation/test_slam_probe.py`: `41d9f3e99e6a16ae660bef8d15e86528ff1de051357a6bc9e92e986936b8de0e`
- `mapping/cuvslam/track_rgbd.py`: `9bd0248f3202da095b712cd422e13ff1146c2e2908a90ba9c0562137ef5e62d3`
- `mapping/cuvslam/test_adapter.py`: `f18532bc7c7cfe810466317b245b511def9ac09e6a3fac740cb671684a53f476`

`RgbdSlam.track` now rejects all-zero depth before advancing timestamps or calling the SDK, clears the last visual pose, and leaves accepted SDK history untouched. Production replay catches that rejected input and exports an incomplete report, including safe null timing statistics when the first frame fails. Guarded diagnostic export now uses the shared `optimized_map_poses` helper with initial wheel/IMU plus mount and preserves original visual poses separately. The regression verifies that this changes only the initial gauge and preserves later vertical increments.

Independent focused verification:

- SDK wrapper, evaluation `test_slam_probe.py`: **4 passed** in 0.013 s.
- SDK wrapper, adapter `test_adapter.py`: **9 passed** in 0.025 s.

Verdict: Ship these diagnostic/input-integrity changes. No blocker remains in the reviewed scope.

Not checked: GPU behavior, the genuine fixed-neck probe results, whole-room loop quality, floor-free artifact geometry, or hardware. A nonzero depth pixel count still does not establish observability; this fix prevents one proven invalid input class only.

## Fixed-neck R1 diagnosis and the next controlled test

R1 completes 150/150 but remains below the clean-map criterion: first-pose translation RMSE 6.71 cm, endpoint 10.60 cm; accumulated rotation RMSE 1.95°, endpoint 3.65°. Path ratio is .9211. Local odometry and optimized SLAM are essentially identical; no LC occurred. The remaining error originates before pose-graph correction. Post-fit 3.96 cm ATE is not the primary acceptance measure.

Independent CPU RGB/depth reprojection check used 29 pairs, three frames apart (about .20 s). Shi–Tomasi corners (500 maximum, quality .01, minimum spacing 8 px), forward/backward LK consistency <.5 px, and source nonzero measured depth gave **median 50 initial RGB corners but only 10 depth-supported surviving matches per pair**, range 0–23. For 244 retained matches, reprojection through the evaluator's exact relative camera poses and configured K has median/.90/.95 error **.230/.958/1.570 px**. Thus the input is still geometrically weak despite containing stair/furniture edges; it is not richly textured. These measurements are evaluator diagnostics, not a new estimator. They do not establish a particular minimum track threshold.

Independent sparse CPU `mujoco.mj_ray` depth check (no renderer, GPU, or tracking) compared existing fusion depth at a 16 px grid against environment intersections. It used only evaluator body-plus-mount camera poses and excluded robot geometry exactly as sensor rendering does. Truth remained entirely in evaluation; no corrected depths were saved or fed into tracking.

| Frame | Supported rays | Median predicted/true optical Z | Mean absolute Z error | 90% absolute Z error |
|---|---:|---:|---:|---:|
| 0 | 555 | .9752 | 7.48 cm | 21.98 cm |
| 30 | 505 | .9624 | 9.17 cm | 29.87 cm |
| 60 | 507 | .9600 | 8.63 cm | 26.28 cm |
| 90 | 523 | .9606 | 8.11 cm | 22.04 cm |
| 120 | 707 | .9952 | 6.02 cm | 12.46 cm |
| 149 | 779 | .9970 | 5.38 cm | 12.19 cm |

This supports a depth geometry problem, not a single global 8% scale factor: early depth is biased short and its per-pixel ratios have wide tails, while later median depth is nearly metric. A global rescale would correct neither the nonrigid depth shape nor accumulated angle and could damage later frames. The configured camera pinhole geometry follows the renderer analytically; RGB reprojection provides no evidence of a gross focal error. The half-pixel ToF-box convention and ~.3° calibration error are too small to assume as the sole cause.

Recommended next measurement-only ablation: retain the same RGB, timestamps, K and SDK settings, but mask out depth from broad/mixed ToF zones and depth discontinuity boundaries, without changing any retained depth values. Use existing hist sigma and ToF point association; compare coverage and the same first-anchor/RPE metrics. This tests whether misleading per-pixel shape in mixed return regions drives scale and angular drift. It must retain enough spatially distributed features; report the reduced support and treat earlier failure as a negative result. Do not pick a threshold from truth residuals or increase the support tolerance. A separate richer-view/fixed-neck capture then tests the demonstrated low corner support, with genuine sensor data rather than added image texture in postprocessing.

A high-value calibration check before changing K is a sparse evaluator ray/reprojection test on known image edges over the image, not fitting K to the drifting SLAM trajectory. Fit-to-trajectory calibration would confuse depth errors with camera intrinsics. No ideal depth or truth pose should enter the production estimator.

CPU true-depth refinement by surface and RGB corners (same six frames, unchanged input):

- Floor grid rays: n=1,601, median fusion/true Z .9500, signed median −3.82 cm, MAE 9.18 cm, p90 absolute 27.43 cm.
- Other-surface grid rays: n=1,975, median ratio .9983, signed median −.49 cm, MAE 5.65 cm, p90 absolute 15.80 cm.
- Depth-supported RGB corners: n=40 across all six frames; **none on the floor**. Median ratio .9620, signed median −7.35 cm, MAE 15.72 cm, p90 absolute 31.22 cm. Frame-wise raw/supported corner counts: 65/16, 52/8, 53/7, 45/9, 25/0, 13/0. These independent image corners differ from the SDK's internal feature detector, so they diagnose weak textured support rather than assert the exact SDK match count.

These results make a constant global positive-scale correction an unproven diagnostic: floor, nonfloor, and corners have different errors, and later inputs are closer to metric than earlier ones. It must be trained on measurement-only ToF associations and validated on held-out zones/frames, without tuning to evaluator truth or SLAM path length. Keep bias/scale bounded, require a well-conditioned fit with enough depth range, retain the unchanged final ToF consistency mask, and fail to identity on insufficient fit. A held-out median metric alone is insufficient: also report signed bias and tail errors, because a scale can reduce a mean while damaging an edge or low obstacle.

Coordinator-authorized oracle-depth diagnostic is valuable once to isolate the remaining depth-shape effect. Keep the exact same RGB, native timestamps, K, SDK settings and **original supported-depth mask**; replace only supported positive Z values by evaluator depth in a separately named oracle-control dataset/run. Never promote or reconstruct its output as measured mapping. Label its report `ground_truth_depth_used=true`, maintain separate provenance, and compare first-anchor translation/orientation and path ratio with every measured run. If oracle control removes most drift, prioritize depth calibration/shape; if it does not, prioritize weak RGB constraints/viewpoint. A dense oracle control would additionally change coverage, confounding the two effects; the original-mask control isolates depth values first.

## Focused review package 3 — ToF scale diagnostic

What this change does: Fits one positive per-frame scale from narrow checkerboard training zones, then rejects it on held-out per-zone residuals before support filtering. It preserves missing prediction pixels, applies the existing support function, and writes selected candidates only if every selected frame passes. Its report explicitly withholds production approval and true geometry/edge/tracking acceptance.

Independent verification: `test_diagnose_tof_scale.py` **4 passed** in .269 s through the SDK environment wrapper, CPU only. The tests cover a known global scale with full extrinsics, missing pixels, broad/mixed or insufficient zones, and held-out values excluded from fit.

Finding: the planned *bounded* scale is currently merely positive (`evaluation/diagnose_tof_scale.py:102–107`). Arbitrarily small/large finite candidates can be accepted if they fit the zone tests. Sent to the coding lead: freeze a meaningful diagnostic range and reject out-of-range fits, or explicitly state that no bound is present and keep it strictly nonproduction. Do not choose bounds from evaluator truth.

No additional implementation blocker found in the diagnostic-only scope. Pixel counts alone do not establish edge retention; the output report correctly says geometry/edge/tracking validation has not passed. Production depth behavior remains unchanged.

Further R1–R3 diagnosis: endpoint first-anchor world rotation vectors xyz(deg) are [2.492,.645,−2.588], [1.011,−.668,−2.227], [1.400,−.093,−2.162]. Consistent negative yaw drift plus variable roll cannot be certified away by a metric-scale adjustment.

Not checked: the new scale command on real selected frames, the oracle control, or any GPU tracking result from the diagnostic candidate. Awaiting coordinator results before recommending depth behavior changes.

Scale diagnostic final review: finding resolved by predeclared [0.1,10] bracket and rejection, without clamping. New mixed-region control veto prevents a biased training region from justifying a scale that damages metric held-out regions. Independent **6/6 tests passed** in .348 s. Snapshot hashes: `diagnose_tof_scale.py` `b80aeb8db67a06a52c5686809ab1cac340d7f0acd1c862e90ffbd0bf7188ac59`; `test_diagnose_tof_scale.py` `cd814021d65f6db31874def96e8ff312a84f90defd84e977e0f75edd9780c7dd`.

Verdict: Ship the diagnostic only. No remaining code blocker in this reviewed scope; no production scale change approved. Coding lead reports the actual old 12-frame probe accepts only 1/12 and all 12 fixed-neck samples reject insufficient single-surface training coverage. Those results were not independently rerun; the report paths are `/tmp/tof-scale-old-20261115-coding/quality.json` and `/tmp/tof-scale-fixedneck-20261115-coding/quality.json`.

Coordinator reports same-fusion-mask oracle control completes 150 frames with first-anchor RMSE 5.55 cm, endpoint 7.16 cm, orientation RMSE 2.24°, endpoint 4.11°, estimated/true path 1.14785/1.19149 m. Accurate positive depth values alone do not eliminate accumulated angle. The next most informative control is full oracle coverage, compared directly to masked oracle with identical RGB/timestamps/K/settings. This changes one variable (depth coverage) and precedes full raw DELTAR, which would change both coverage and depth accuracy. Oracle controls remain explicitly truth-depth diagnostics, with no SDK map, obstacle export or promotion.

## Focused review package 4 — physical ground classification and layer PLY export

What this change does: Classifies pixels using synchronized physical wheel/IMU plus camera mount when present, while placing every retained measurement with the unchanged final optimized SLAM camera pose. Exports separate binary ground/obstacle/uncertain PLY layers after full-pixel backprojection and independent 3 cm voxel sampling per class. Original depth and final pose hashes are retained.

Reviewed snapshots: `reconstruction/segment_scan.py` `b734e0ac96994a8c7aeeb2f8ad552c5c254cbad90c9800cc963de11bcd721deb`; `reconstruction/test_ground_segmentation.py` `609b559f47fbb0c9b55ffd6be73228269ba4ab92052f071040cad08d9eab1e46`.

No blocker found. Physical pixel labels and camera-local planes are independent of the optimized map gauge; the new test correctly moves the final map floor up by 1 m yet retains physical ground classification and exports it at the optimized location. It never flattens the pose or labels a globally low point as floor merely by exported z. Mixed or partial physical metadata is rejected before writing. The named legacy fallback remains explicit and retains its stronger flat-Z-up prerequisite.

The actual 150-frame artifact was independently reread, not merely trusted from its report:

- All original/exported timestamps and final camera matrices match exactly.
- Recomputed depth SHA-256 `12560ddf67139f05f8e0c9cc0198095f0923e6e51b4c235582ea2ba27fb200e6` and pose SHA-256 `a7f070d509e0420a0673127dab15db7091cdc744a0879d24efebb47a287581ff` match the report.
- Recounted valid labels: uncertain 13,488,921; ground 1,155,810; obstacle 8,748,524. Invalid/range-excluded pixels: 22,686,745, all uncertain. No invalid depth is backprojected.
- Independent binary PLY parsing exactly matches each corresponding NPZ array as little-endian float32: **26,622 obstacle, 25,281 uncertain, 1,619 ground points**, all finite. Layers are partitioned by physical class before independent voxel sampling, preserving different classes in the same spatial voxel.
- The 3 cm obstacle and 2 cm uncertain fixtures remain protected; the new off-old-sampling-grid fixture proves a one-pixel point in each class survives. No stride-before-voxel thinning or smoothing remains in this exporter.
- Report is explicitly `diagnostic/unvalidated`, says uncertainty is not free, and withholds navigation/clearance validation. `obstacles.ply` contains only `OBSTACLE` labels; this is a classification invariant, not proof that every physical floor pixel was correctly classified.

Independent CPU verification: **10 ground/export tests passed** in .277 s; **14 affected navigation grid tests passed** in .132 s. The real exporter reports 15.48 s for 150 frames. Memory is bounded to 32 full frames plus accumulated voxel layers; this is reasonable for the current offline diagnostic workload and does not involve CUDA.

Verdict: Ship the diagnostic exporter and its actual separate layers. This does not approve the SLAM map, certify physical classification accuracy, or authorize navigation. No production pose/depth, support mask, or retained uncertainty was changed.

Not checked: viewer rendering, obstacle-vs-simulator truth accuracy across the real room, full-room runtime/memory, or hardware. At most one sample of a given class per 3 cm voxel survives; the export preserves class occupancy but not every geometric detail within a voxel.

## Focused review package 5 — classified PLY RViz viewer

What this change does: Loads actual binary PLY layers with exact exporter header/size/count validation and required diagnostic provenance, publishes isolated `/rb_classified/*` topics, and displays obstacles plus a persistent diagnostic marker by default. Ground/uncertainty remain independent checkboxes; hidden uncertainty is explicitly not free. The launcher validates first, clears inherited Python paths, owns its publisher PID and temporary RViz config, and cleans them on exit.

Reviewed hashes: `viewer/classified_scan.py` `097dcafee6220414cd87fd70e612e8e29f70904419855a38bfeec91f848ab087`; `run_classified_rviz.sh` `c939191e7b264ececa537ef32f2e66562eff7a7a18389914efadba805c211005`; `classified.rviz` `f7bd04a1ba68cb853b252c9626428776e80c485c96e0396d3a828980d8a71864`; `test_classified_scan.py` `87236aaa7cff67b0c68ca69d51f70ef46761ff9e26897c89176e0a481eadd444`.

Independent checks: `bash -n run_classified_rviz.sh` passed; **3 loader/config CPU tests passed** in .052 s. They cover empty layers, wrong reported count, truncated payload, NaN, wrong provenance, checksum disagreement and unexpected PLY format. No GPU or GUI process was started by this reviewer.

**Must fix for standalone usability: register the map frame.** The publisher initially only sends map-frame PointCloud2/Marker messages and no TF. An empty installed `tf2_py.BufferCore` independently gives `can_transform_core(map,map)=true` but `_frameExists(map)=false`: points can render while RViz Global Status still warns that the fixed frame has no TF data. [Official RViz `updateFrames` implementation](https://raw.githubusercontent.com/ros2/rviz/rolling/rviz_common/src/rviz_common/visualization_manager.cpp) explicitly checks fixed-frame existence and reports this condition. Existing other viewer TF can mask it. Send an identity static transform from map to a uniquely named display child; do not override an external world-to-map or change scan coordinates. Sent to coordinator/coding lead; coordinator independently confirmed the warning in actual RViz.

Verdict: fix standalone TF status before calling this a standalone viewer. Loader, source scope, topic isolation, launcher lifecycle and diagnostic semantics otherwise have no blocker. Coordinator owns GUI layout/visual verification.

Screening reproducibility clarification: earlier six-frame counts 65/16,52/8,53/7,45/9,25/0,13/0 included CPU evaluator ray hit/range exclusions. Pure measurement-only screening with the frozen detector gives **65/22,52/12,53/8,45/13,25/0,13/0**. Exact settings: `cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)`, `cv2.goodFeaturesToTrack(gray,maxCorners=500,qualityLevel=.01,minDistance=8)` and all remaining defaults; nearest pixel `np.rint(points[:,0]).astype(int)`, depth uint16 >0. Restricting to 20<depth_mm<4000 gives the same counts on those frames. No LK, temporal match, or truth exclusion belongs in the new preview screening. Spatial coverage uses a frozen 4-column ×3-row image grid.

Viewer finding resolved in source: static identity `map -> rb_classified_origin` is published by a retained `StaticTransformBroadcaster`; cloud/marker point coordinates and frames remain unchanged, and no external world-to-map is overridden. The diagnostic text is now three compact lines with font size capped at .07 m. Updated `classified_scan.py` hash: `160586aac54033e401769e056cdcc3e1a451ad4e80a2cefc0e04f6214b67b4b1`. Independent **3 loader/config tests passed** again in .035 s after the change. Coding lead reports an isolated ROS-domain TF/status smoke test; coordinator owns the actual GUI Global Status and text-layout check.

Verdict: Ship the viewer code for diagnostic inspection. No remaining source-review blocker. GUI appearance and standalone Global Status are awaiting coordinator verification, not claimed by this reviewer.


## Combined CPU release review — current affected components

Independent combined verification on 2026-11-15: **148 tests passed** with GPU visibility disabled. Counts: adapter 9, live IPC/history 13, reconstruction 34, evaluation 22, navigation 67, classified viewer 3. Navigation/viewer used the ROS Python environment with `ROS_DOMAIN_ID=184`; adapter used the cuVSLAM Python wrapper; live/reconstruction/evaluation used the DELTAR environment. `bash -n` passed for both affected RViz launchers. The navigation suite used its existing withdrawn Tk fixture and isolated ROS nodes; this reviewer did not start a simulator, GPU tracking, or an RViz GUI.

The updated evaluator now reports first-anchor translation/rotation p95 and maximum, path-length ratio, one-second RPE p95/max, maximum input interval, and actual relative-pair horizons. Its empty/short stationary cases serialize finite JSON with null path ratio when the true path is zero. Independent fixed-neck R1 read-only scoring reproduced translation p95/max 0.10756800697029108/0.10828897148121348 m, rotation p95/max 3.4870100595906233/3.6496661942154107 degrees, and path ratio 0.921134066751383. This is still Gate A failure, not an accepted map. Reviewed snapshot: `evaluate_slam_map.py` SHA-256 `02026e7ad4ce5acf1425d4b16ad1db3b6e16f84328b25bfa5be0a2c7fb0fb7e6`; `test_slam_probe.py` `11bf3ad480e668c7f72e8940a06c244c782ad8fec338b7624bb8fdf33cd0eadf`.

### Remaining concrete evaluator requests

1. **Incorrect oracle truth-scope sentence.** `evaluate_prepared` hardcodes “Evaluator only; no truth enters tracker or reconstruction.” Both actual `evaluator-only-oracle-depth-control-20261115/evaluation.json` and `evaluator-only-full-oracle-depth-control-20261115/evaluation.json` contain this false sentence although their source and live sequence correctly declare `ground_truth_depth_used=true`. Propagate declared oracle provenance and explicit diagnostic/nonaccepted status, or reject oracle controls from this evaluator; add a regression. Do not infer provenance from a directory name. Sent to coding lead and coordinator.
2. The frozen contract requires one-second pairs matched within one native interval (<=0.07 s), whereas the evaluator still uses 0.10 s. Make the intended tolerance explicit and test the near-boundary case, plus an interior-missing-frame/source-coverage fixture. Existing R1 has exactly the same 136 pairs at either tolerance, with actual horizons 0.94..1.00 s, so no GPU rerun is needed to correct this report.

The product JSON correctly says `IN PROGRESS: NO ACCEPTED CLEAN MAP`, marks every run `accepted_map=false`, and labels the static repetition and oracle controls as diagnostics. The measured full-prediction diagnostic is not a production tracking-depth change. Documentation continues to withhold native navigation/validated-map claims. The actual obstacle-layer export and original hashes remain measured, as independently verified in package 4.

Related scope concern sent to coordinator only: exporter `segment_scan.run` does not reject a source sequence declaring `ground_truth_depth_used=true`. Its prepared-source fallback could accidentally pair an oracle-derived trajectory with original measured depth, then emit `ground_truth_geometry=false`. This has not occurred in the reviewed actual export. A small provenance rejection before any output creation would enforce the existing no-oracle-export contract; no oracle export should be executed to demonstrate it.

Verdict at this checkpoint: CPU behavior passes; corrected evaluator provenance and the intended horizon/coverage regression remain requested. No map, loop closure, physical floor accuracy, or navigation acceptance follows from these passing tests. Not checked: coordinator-owned new capture, GPU results, GUI screenshot/layout, hardware, or held-out measured geometry quality.


## Evaluator/export follow-up — resolved and independently retested

Independent focused verification after the fixes: **23 tests passed** (9 probe/evaluator, 11 ground/export, 3 viewer). ROS-dependent viewer checks again used domain 184; both RViz launcher shell syntax checks passed earlier. The 148-test combined checkpoint preceded these four new regressions; this is an affected-slice rerun, not a claim of a new full-suite run.

The evaluator now defaults to <=.07 s RPE matching, derives a stricter value from faster prepared source cadence, reports actual horizons, and explicitly identifies complete, empty, accepted-prefix, interior-gap and truncated-window coverage. Useful prefix scores remain available without claiming complete source coverage. New tests prove interior omissions remain visible, .08–.10 s mismatches are not accepted as native one-frame matches, and declared oracle input is preserved in truthful diagnostic provenance. Existing oracle report files still need coordinator refresh; corrected source code does not retroactively change their stale text.

The exporter rejects declared oracle-depth, oracle-pose or `not_for_mapping` flags from the sequence/source/tracking records **before creating output**. Its synthetic regression pairs ordinary measured depth with each declared forbidden provenance source; every case rejects and leaves no directory. The existing measured diagnostic export was not regenerated and remains unchanged. No remaining source-review blocker in this bounded patch. Snapshot hashes: evaluator `5d4250903add64a607a2e4e40db731366f090185024ab1d6f0126cdb73fd27cc`; evaluator tests `920aaf92b074669d7c7d008e078edd0d53bca7a4aa73f21ef7d044a377631101`; exporter `9195bc6a6c8235c78d87c5d41c02955ffdddc5760f23222c50efa26db6a24e59`; export tests `9d97e8d257df439835bef67ea229e3a17176afcd5412a6f8d0153c7811836f2a`.

## Independent held-out physical floor quality audit — Gate C fails

Machine-readable evidence: `evaluation/clean_slam_floor_quality_review_20261115.json`. Labels, depth, poses and PLY files were read only. CPU `mj_ray` scored the original scene at exact captured robot/joint states; the true camera matrices independently matched the isolated prepared truth to <1e-10. No renderer, GPU, simulator stepping, tracker or reconstruction was run. Robot geometry was excluded using the same sensor visibility groups; first visible scene hits enforce occlusion. No depth-agreement filtering discards difficult valid predictions.

Frozen sampling: frame indices **[7,22,37,52,67,82,97,112,127,142]**, every eighth image pixel from (0,0): **48,000 samples**. There were 24,162 valid measured samples; 23,535 had a visible true hit in the .02–4 m scoring range, and 627 valid measurements had unscored truth. The floor denominator includes every valid visible floor sample, including uncertain labels.

- Removed-floor precision: **1,339 / 1,352 = 99.038%**, below the 99.5% target. The 13 wrongly removed samples hit the bottom of walls.
- All-valid-floor recall: **1,339 / 10,640 = 12.585%**, far below 90%. Of the remaining valid physical floor samples, **7,970 were uncertain and 1,331 were obstacle**. Three held-out frames had zero confirmed ground.
- Visible-floor measured-depth completeness: **10,640 / 15,522 = 68.548%**. Invalid depth was not excluded from this separate completeness denominator.
- Visible nonfloor surface samples at least 2.5 cm above the physical floor: **12,474**. All 12,474 stayed non-ground/blocked; **8,683 (69.609%)** entered the obstacle class and **3,791** remained uncertain. This is a surface-hit-height stratum, not proof of recall for 2.5 cm object instances; the existing synthetic low-object fixtures remain separate.
- Uncertainty among valid truth-scored samples: **12,026 / 23,535 = 51.098%**. Hidden uncertainty cannot be counted as free or absent floor.

Actual obstacle PLY leakage was evaluated independently of the saved layer NPZ: all **26,622** binary PLY vertices were exactly recovered as their first original measured frame/pixel per 3 cm voxel. Every recovered coordinate matched bit-for-bit. CPU source-camera rays across all contributing frames then found **3,301 vertices whose first visible physical surface was ground**, **14,283 nonfloor**, and **9,038 outside the true scoring range or without a usable hit**. Confirmed physical-floor leakage is therefore at least **12.40% of all obstacle vertices**, or **18.773% of the 17,584 truth-scored vertices**. This assignment uses physical source visibility, not drifted map Z, and does not flatten or realign geometry.

Causal height scoring distinguishes mount/IMU validity from depth shape: on all 10,640 valid true-floor samples, the unchanged physical wheel/IMU+mount frame gives evaluator-only optical-depth floor height median **0.545 mm**, p95 **1.245 mm**, maximum absolute **2.415 mm**. With actual measured depth, floor height median is **11.499 mm**, p95 **67.996 mm**. For the 1,331 floor samples labeled obstacle, actual measured height median is **52.952 mm** (p05/p95 **27.210/89.440 mm**), whereas optical-truth depth in the same physical frame gives median **0.943 mm**; their depth is underestimated by median **359.240 mm**. This isolates a real spatial depth error that turns physical floor into apparent raised geometry. It does not justify classifying by optimized map z=0 or increasing floor tolerances to swallow low obstacles.

Verdict: **Gate C fails** on precision, recall and actual obstacle-layer floor leakage. The clean-looking obstacle view cannot be described as a floor-free accepted map. The audit is bounded to this session and visible surfaces; no hardware, full-room completeness, repeated-map seam accuracy or navigation quality is approved.


### Floor audit extension — exact support paths and classifier losses

The same ten held-out frames now include a byte-identical reconstruction of the unchanged prediction → additive ToF correction → final support output. Quantized output matched all original stored depth pixels before any branch scoring. All 1,331 sampled physical-floor false obstacles passed: **709 normal mean-tolerance only; 618 both normal and slope; 4 slope only**. Only **4** relied on a slope override of the nearest-return veto. Disabling slope support alone therefore cannot fix the other 1,327. For 50 false obstacles, the depth-dependent predicted ToF association differed from the association of the actual physical hit; all true projections remained inside ToF.

CPU 16×16 first-hit rays for each implicated ToF zone found **1,099 / 1,331** false obstacles associated with >=99% physical-floor zones, and **232** with mixed floor/nonfloor zones. This is not primarily a mixed-object occlusion problem. The JSON retains each frame-zone's measured mean/sigma, corrected-ToF residual distribution, independent zone hit distribution and selected nearest-cluster distribution, plus all 1,331 sampled false-obstacle pixel records.

**Concrete integration hypothesis:** `capture/capture_mujoco.py:71–82` defines histogram mean/sigma from the **closest significant cluster**, while `reconstruction/depth_support.py:34–37` compares a predicted **whole-zone** slope distribution to that selected-cluster mean/sigma. Local mean anchoring at lines 107–115 likewise moves nearby predicted geometry toward the selected return. Those quantities have different semantics on a continuous oblique floor. At held-out frame 7, zone 42, all **256/256** true ToF rays hit floor: the full-zone true optical-Z mean/sigma is **1.5721 / .3163 m**, but the closest significant subset contains **80 rays** with mean/sigma **1.2331 / .0593 m**; the recorded sensor mean/sigma **1.2396 / .07235 m** agrees with that selected near subset. The admitted short prediction is close to the selected mean but turns **40** sampled physical-floor points into apparent obstacles. Whole-zone truth mean differs from the selected recorded mean by about **.332 m**. This is a testable measurement-association mismatch, not evidence that the sensor or physical floor is wrong.

Minimal next controlled test: synthetic analytic oblique plane and two-surface depth arrays through the existing nearest-cluster sensor reducer and unchanged support, asserting which statistic each consumer means. Freeze selection bins/thresholds and include fronto-parallel positive control. Compare nearest-cluster-aware residuals/anchoring as a diagnostic on measured training zones with independent held-out zone/frame scoring; do not replace sensor records by whole-zone truth, erase physical floor using map Z, or loosen low-object thresholds. The direction of any production fix must respect actual sensor closest-return semantics.

Uncertain true-floor failure accounting: **7,970** samples total; **3,202** lack confirmed planes. Of **4,768** with confirmed planes, **2,850** fail physical height ±1.2 cm, **1,300** fail plane residual, **2,684** fail local normal, **202** fail local smoothness, and **267** pass center checks but fail erosion. These failures overlap. Zero uncertain samples unexpectedly passed every check after erosion. Improving temporal confirmation alone will not repair bad measured height/normal.

### Independent original-material control review

The isolated scene variant changes only two original-source texture/material assets and the corresponding sofa/table appearance. The new structural test removes exactly those allowed attributes/assets then recursively matches the original XML; source DAE image links, RGB pixels and retained OBJ UV indices are checked. Independent **2 tests passed** in .087 s. Independent CPU compilation with the existing `inertia=shell` compatibility stripping compared **34** geometry/body/dynamics/joint/collision/mesh/camera/sensor/actuator arrays exactly, with 97 geoms and 2 cameras unchanged; only texture/material counts grew 1→3. No capture, renderer, GPU or simulator stepping was run.

Hashes: original scene remains `3687887b7892988abb688dd23f4e8947eff3820b9189c809f825dcc713b705a5`; material-control scene `6056138c61708e87ee0fe837899325c0131822dff774f007b211692bfee520c2`; test `ec74a770a8c9fa26e677b1a74be2c41c152b9a0f9634638f33924d349433e074`. No structural blocker for a clearly isolated appearance control; it cannot establish quality on the unchanged original room or authorize map promotion. Coordinator reports the compact viewer screenshot now has Global Status OK and readable tokens, and the seven old evaluator reports have been refreshed truthfully without rerunning GPU work; these GUI/report refreshes are coordinator verification, not new measurements by this reviewer.

Follow-up read-only check: both original-mask and full oracle control `evaluation.json` now independently confirm truthful oracle scope, `ground_truth_depth_used=true`, and `not_for_mapping=true`. The stale-report finding is resolved.


## DELTAR spatial association contract — independent preimplementation review

Source inspection confirms an integration defect: `utils/dataloader.py:18–32` derives maximum patch size and a bounding rectangle from **all** zone rectangles. `models/fusion.py:71–85,129–145` combines bounds across the batch, crops one bounding box, interpolates it and divides it uniformly into 8×8 zones. The per-zone calibrated rectangles are not used to gather each zone. Invalid histogram masks are applied only after cross-attention, so an invalid rectangle can still move valid histogram features to the wrong image location.

Independent CPU reproduction on unchanged native input frame 75: invalid zone 33 has rectangle [238,39,304,103], while every valid rectangle begins at x>=103. The legacy stride-4 bbox begins at feature x=9 (image x=36), producing valid-zone center errors median **25.685 px**, maximum **79.952 px** against individual calibrated rectangles. Mutating only invalid rectangles changes every valid zone's gather position despite unchanged valid RGB/ToF measurements. There are 20 invalid zones in that frame. This is an input contract issue independent of model weights, pose estimation or truth.

Simply excluding invalid rectangles from the bbox is insufficient: missing outer rows/columns would shrink the bbox and shift zone identities; valid depth-dependent projected rectangles are also nonuniform. Review requirements sent to coding/planning: preserve explicit zone order; gather image features by each valid calibrated rectangle with a stated feature-center coordinate convention; inversely place the same zone result into its original footprint; normalize overlaps; avoid all updates from masked, empty or fully out-of-bounds rectangles; keep batches independent and checkpoint learned parameters unchanged. Include a uniform-grid equivalence positive control, invalid-rectangle mutation regression, missing edge zones, overlap/OOB/all-invalid cases and two-item batches with different footprints. Root owns any later same-input inference/three-repeat ablation.

Separate semantic clarification: DELTAR's `get_hist_parallel` training path chooses the strongest contiguous histogram cluster (`utils/dataloader.py:103–113`), not whole-zone moments. The physical simulated sensor intentionally chooses the closest significant cluster. The previously documented whole-zone/selected-cluster mismatch concerns the **downstream support consumer**, not a claim that DELTAR was trained on whole-zone moments. Neither statistic should be silently replaced to improve a test result.

Actual forward-path regression independently reproduced on CPU, without checkpoint inference: `TransformerFusion(8,(120,160))`, zero positional embeddings, zero RGB features, histogram features containing each zone identity, and a deterministic attention stub returning an independent clone of the zone-ID constant. On frame 75, moving only invalid rectangles' x bounds left by 64 pixels changes **5,802 / 19,200** output feature pixels, maximum value difference **56.0**. Input histogram features remain bit-identical. Thus the defect changes the real fusion output, not only a diagnostic center metric. No GPU was invoked.


### Spatial adapter final independent review and frozen prepared-input A/B

The isolated calibrated adapter owns each existing feature center `(index+.5)*stride` inside its valid half-open rectangle, fuses only that zone's histogram, and averages overlapping updates on the same cells. It avoids resampling, bbox inference, padded queries and new learned parameters. Legacy storage/alias semantics before the first image attention layer are preserved. Exact aligned uniform-grid equivalence passes with real attention layers at strides 4/8/16. Independent latest **7 CPU tests passed** in .420 s: legacy-red/new-green invalid-box regression, invalid histogram/geometry independence, missing edge zone identity, overlap mean, partial/complete OOB, finite gradients, narrow empty-center boxes, batch equality/permutation, and actual all-invalid full forward matching the real image-only layers.

Original `nyu.pt` independently loads on CPU with **strict=True: all 968 keys match**, three calibrated fusion modules, unchanged **19,701,246** learned parameters. No inference was performed by this reviewer. Across all 150 native H5 inputs, **7,565** valid frame-zones have nonzero feature membership at every scale: stride4 208–272 cells, stride8 49–81, stride16 9–20. Runner freezes input H5/index/checkpoint/adapter hashes before/after; production model files are unchanged. Adapter hash `829e3f2e2eca14e1bb89bcec22f20d1fe31214589a5b2a266c685e20c8d3f4b4`; runner `10f5c6ca15f34846c1001077cdff64442871f328d982e9a3b77cd8359ee3ff46`. Verdict: cleared for the bounded diagnostic same-input A/B, with no production promotion.

Coordinator then ran both prediction modes and prepared complete physical-label history. This reviewer independently compared **46,080,000 baseline depth pixels** against the original and **all150** baseline ground-label/plane/confirmation/source records against the old diagnostic export: exact equality, including NaN plane positions. A helper dtype discrepancy was corrected by coordinator before scoring; both sides now use the original float32-meter classifier path.

New reusable read-only scorer `evaluation/review_floor_quality_prepared.py` accepts the two prepared folders plus explicit original truth source and a new evidence path. It validates K and every one of 150 source identities/timestamps/odom/mounts, reads existing labels, uses the same frozen ten indices/stride8/ray visibility/range rules, checks watched-file hashes before/after, and writes only its new report. It never classifies, runs SLAM, renders, steps physics or exports PLY. A known five-sample denominator control passed; actual baseline counts exactly reproduce the prior floor audit. New report: `evaluation/clean_slam_spatial_floor_comparison_20261115.json`.

Calibrated versus baseline on unchanged visible truth sampling:

- Removal precision **1181/1184=99.747%** versus **1339/1352=99.038%**. The isolated precision threshold passes, but floor recall worsens to **1181/10747=10.989%** versus **1339/10640=12.585%**.
- Physical floor labeled obstacle falls **1331→1119**; uncertain floor rises **7970→8447**. Measured-floor completeness changes **68.548→69.237%**. Overall truth-scored uncertainty increases **51.098→56.733%**.
- Visible >=2.5 cm nonfloor-surface retention remains **100% blocked/non-ground**. Obstacle-only fraction falls **69.609→61.333%**; uncertainty still blocks and does not count as free.
- Raw predictions evaluated on the identical **15,522 visible floor rays**: optical-depth median error **−31.196→−15.126 mm**, physical-height median **8.341→4.665 mm**, p95 absolute physical-height **106.072→105.284 mm**. Bias improves but tails remain large.
- On the **10,481 common supported-floor samples**, raw physical-height median improves **8.470→5.167 mm**, while raw p95 remains **72.477→72.417 mm**. After unchanged ToF correction/support, median height is **11.606→10.820 mm**, p95 **66.612→66.763 mm**. This paired comparison prevents coverage selection from hiding errors: the large supported floor tail is essentially unchanged.
- With the original frozen RGB detector at indices [0,30,60,90,120,149], depth-supported corner counts change **[22,12,8,13,0,0]→[20,20,16,14,0,0]**. Both late views still have zero supported corners despite nonzero depth images.

Verdict: spatial mapping is demonstrably corrected and improves raw bias, but **Gate C remains failed**. Downstream nearest-cluster/whole-zone anchoring semantics remain a separate primary investigation. A single guarded tracking screen can test whether improved mid-segment feature support helps actual tracking; if sensible, complete all three predeclared repeats. No obstacle PLY or accepted map follows from this prepared-input audit. Not checked: new GPU tracking behavior, hardware, loop seams or navigation.


### Independent calibrated tracking Gate A — all three repeats

All three completed guarded run directories were independently scored directly from saved camera matrices and isolated prepared body truth/mounts, using separate numerical code rather than trusting their `evaluation.json`. Every immediate/local/final history contains exactly all 150 requested identities in order, with exact nanosecond timestamps, rigid finite matrices, maximum source gap 0.0700000000002 s, 136 one-second pairs and actual horizons .94–1.00 s. The .07 s pair tolerance is enforced. Tracking reports complete, no rejected/substituted frame, no SDK map save, no LC events; prepared provenance has no oracle depth/pose. R1/R2 published values matched independent math within 1e-10; R3 evaluation was not yet present at initial read.

| Repeat | First-anchor position RMSE | Max position | Max angle | RPE1s translation / rotation | Path ratio | Gate A repeat |
|---|---:|---:|---:|---:|---:|---|
| R1 | 4.5431 cm | 7.4027 cm | 2.63868° | 1.0620 cm / .42444° | .94445 | Pass |
| R2 | 4.0112 cm | 6.3706 cm | 1.98816° | 1.0851 cm / .36012° | .95649 | Pass |
| R3 | 4.6794 cm | 7.3872 cm | **3.32365°** | 1.1435 cm / .45010° | .94951 | **Fail** |

Overall **three-repeat Gate A FAILS** because R3 exceeds the frozen 3° maximum orientation limit. Keep the useful improvement over the old input, but do not select the best repeat or loosen the target. All repeats still pass coverage, cadence, translation, RPE and path ratio. No short feasibility result constitutes closed-loop/map/floor/navigation acceptance. Read-only check details are in the ephemeral `/tmp/review_spatial_gate_a_20261115.json`; the persistent numbers above are the review artifact.


Gate A follow-up: R3's published evaluator now matches independent matrix scoring within 1e-10 too. Persistent full evidence is `evaluation/clean_slam_spatial_gate_a_review_20261115.json`; all three trajectories/conditions/horizons and frozen thresholds are retained. Overall status remains FAIL.

### Frozen no-bias ablation — original floor denominator retained

Reviewed coordinator script `/tmp/rb_prepare_no_bias_ablation.py`: same calibrated raw prediction PNGs, exact original ToF mean/sigma/valid/rect data and K, same unchanged `tof_consistency_mask`, same float32-meter physical classification history. Only the additive Gaussian whole-zone bias is removed; input values are masked, not replaced. Independent all150-frame check confirms every **23,040,655** retained positive output pixels equals its raw prediction value exactly. There are **zero confirmed floor planes** across all150 frames.

Read-only report `evaluation/clean_slam_no_bias_floor_comparison_20261115.json` uses unchanged frozen frames/stride/rays/ranges and includes common-mask, each-run and raw diagnostics. The scorer now explicitly retains the FIRST folder's entire valid visible physical-floor mask as a fixed gate denominator, even where the candidate drops depth or leaves uncertainty. This prevents improving recall by discarding difficult floor. Existing own-valid metrics remain separately named and reported.

Baseline calibrated denominator **10,747** visible valid floor samples; no-bias retains depth on **10,503 (97.730%)**, losing244. Candidate labels **all10,747 uncertain**, including missing depth, so fixed-baseline floor recall is **0 /10,747 = 0%**, versus **1181 /10,747 = 10.989%**. On its own support, no-bias sees **10,964** floor samples, still all uncertain. Removal precision is **undefined (zero removed points)**; this cannot be called passing precision or a floor-free obstacle product. All **22,643** valid truth-scored candidate samples are uncertain, including every sampled physical obstacle.

On **10,503 common supported floor samples**, no-bias physical-height median improves **10.907→5.481 mm**, but p95 absolute height worsens **67.412→72.684 mm**; signed lower tail also worsens **−15.767→−51.002 mm**. The raw prediction metrics on the identical15,522 true-floor rays are bit-identical between the two runs, as required. No-bias therefore improves median offset but loses the plane consistency needed for this classifier, rather than repairing the tail or low-object ambiguity.

Verdict: **no-bias fails the floor ablation and is not a production candidate**. Zero obstacle labels result from failed floor confirmation and complete uncertainty, not successful removal. Keep unknown blocked. Do not loosen plane/height thresholds or select only good pixels to rescue the result. No GPU tracking or PLY export was run for this audit by the reviewer.

Scorer denominator safeguard: `fixed_baseline_valid_floor` now reports baseline floor count, candidate retained depth and all three candidate classes on exactly that original mask. Its 90% partial recall gate uses this fixed denominator; candidate-own-valid recall remains separately available. For the no-bias comparison, losing244 baseline floor pixels therefore cannot inflate the reported gate.


### Nearest-return moment/coverage diagnostic review

Independent **5 semantic CPU tests passed** in .317 s and **8 affected geometry tests passed** in .573 s. Pure selector extraction in capture preserves old `zone_statistics` output hist/mask exactly, including RNG state at noise0/.2, against a frozen copy of the old reducer. Selection still uses original 4 cm bins, significant-bin threshold, closest >=20% significant cluster, inclusive original range/bounds and the same noise draw/order. There is no strongest-return or whole-zone semantic change.

The new diagnostic projects RGB predictions in metres through the full OpenGL-coordinate R/t, keeps nearest ToF optical-Z per128×128 raster cell, records deterministic original pixel owners, and reduces each16×16 zone with the exact shared selector. Complete candidate coverage requires256 represented cells; holes are never interpolated. Real CLI mode only scores moments/coverage and checks input hashes; it cannot fit a bias or write depth. The +4 cm correction path is restricted to an explicitly named analytic fixture and is not real-data evidence. No production support/model changes are present. No blocker for the bounded moment-only evidence command; moment agreement still cannot certify dense floor geometry.

Reviewed hashes: diagnostic `7bbe9c5966ec30bcca6b3b7336a21e3a0d766eb400d1f29eca29c3fff2c5019f`; semantic tests `66248359cf647b5d2eab73b61574cf94b4cf86a21081e6d4c558c9aa09f8ae8f`; capture reducer `f7dc6c0adc10865de404699f2ce1e68402fc27838124c9d1334dad0e6198bd41`. Not run by reviewer: real-frame moment report or any candidate bias fitting, GPU inference/tracking, simulator stepping/rendering.

Semantic diagnostic final delta: source-pixel IDs for selected raster winners are now reported. Analytic zero/oblique/mixed/hole fixtures explicitly enable the analytic-only branch, while a separate default-mode biased fixture verifies no real-data correction occurs. Independent final **5 tests passed** again in .329 s. Updated diagnostic hash `85fb91c7497bb8cead9b7804db6983103fc09958e22851889fc364df1a7fd6e4`; test hash unchanged. No new blocker.


### Actual classifier return tracing explains no-bias0/150 confirmation

Evidence: `evaluation/clean_slam_no_bias_classifier_trace_20261115.json`. Read-only `sys.settrace` targeted the actual `GroundSegmenter.classify` code object, line and return locals; no copied classifier, altered branch, threshold, iteration count or seed. One original seeded segmenter (seed51) consumed all150 unchanged frames in order, with exact canonical float32 metres and physical odom/mount. All saved ground labels, camera planes, confirmations and source strings reproduced exactly. Source keyframes were hash-checked unchanged; elapsed3.82 s; no truth/GPU/render/simulator step.

- **142/150** return at classifier line70, insufficient RANSAC support. Every frame has the full2,000-point sampled candidate set, so this is not missing candidate count. **68** have no triplet plane satisfying the fixed geometry checks; **74** have a best plane but fewer than **800** required inliers (40% of2,000). Across9,600 sampled triplets, only **123** pass the triplet geometry constraints. Best fraction where any best plane exists: median **29.55%**, max **44.15%**.
- The remaining **8** frames (indices **[1,10,11,24,60,74,85,127]**) pass the support gate but return at line83 after SVD refinement. All eight violate **only the ±1 cm world-plane-offset limit**: signed distance **+20.732..+42.602 mm**, median **+25.910 mm**. With the upward plane convention this fitted plane is below the calibrated floor. Their tilt **1.322..3.352°** passes8°, and second spatial span **.1026.. .1834 m** passes.08 m.
- Every frame clears history at its geometric rejection. History is already empty on every frame; no timestamp-gap or temporal-jump clear occurs, and no frame reaches the append/confirmation stage. Thus the zero confirmation count is **not a temporal persistence problem** and cannot be repaired merely by waiting for more frames.

Attribution: no-bias raw supported shape does not provide enough geometry compatible with the calibrated floor in these actual classifier trials. The finite64-trial RANSAC budget remains part of the algorithm, so this trace alone does not prove no better plane exists under another search; it establishes the exact current failure and the refined offsets for all successful support hypotheses. Earlier evaluator-only ray scoring placed actual physical floor within2.4 mm of this IMU/mount frame, so the observed2–4 cm plane error should not be excused by flattening optimized poses or raising the floor tolerance (which would erase low obstacles). Selected-return corrections still require independent shape/edge/held-out checks; aggregate moment agreement is insufficient.

Moment-only real report checked read-only: `evaluation/return_moments_calibrated_20261115/quality.json` declares no fit and no candidate artifacts. Development/held-out selected-mean absolute medians are **.0441/.0490 m**, p95 **.1191/.1441 m**; these are diagnostics, not correction acceptance. Final shared capture selector review remains clear: the frozen old reducer regression reproduces hist/mask bytes and RNG state, and the diagnostic CLI has no real-data fitting/output path. No additional tests rerun beyond the already recorded5semantic+8geometry focused checks.

### Actual-baseline owner-gate review — bounded analytic revision only

Read `evaluation/return_owner_gate_fixture_20261115.json` and the current projection/proposal source. The actual nonzero RGB→ToF rotation/translation front-plane fixture is a concrete counterexample to exact source-pixel identity: all56 fully covered proposals improve selected-mean residual from4 cm to numerical zero, yet all56 fail only the pixel-owner test (one edge also loses coverage/association). Depth-dependent parallax changes which RGB sample wins a ToF raster cell on a perfectly smooth surface. This demonstrates over-rejection; it does not demonstrate real-data correction safety.

Planner froze the following narrow replacement before any real-data bias run: fit the ORIGINAL selected predicted continuous surface in continuous ToF angular coordinates before correction; allow changed old/new raster winners only if both original source points belong to that same original surface under the existing inverse-depth fit/span/residual and per-point criteria. Preserve complete coverage, selected-cell mask and cluster association. Outside the selected surface, keep exact original owners. Check the entire affected raster footprint, including neighboring zones. The current target-zone-only sequential owner check can miss neighboring ownership changes and can confuse a previous accepted proposal with the current one; this is a concrete integration issue to cover in the revision.

Required positive/negative fixtures: actual nonzero R/t legitimate +4 cm plane correction must PASS; a foreground/background ownership swap whose mean residual improves must FAIL; off-target/neighbor-zone changes, missing/new rays, selected/nonselected flips and far-tail modifications must FAIL or remain bit-identical. Surface fitting must use the unchanged original samples, not a newly fitted corrected plane that could explain away a visibility error. No additional normal threshold exists in the current source; do not claim one. The flat-plane surface check must not inherit the slope-enable sigma/distribution-variation gates, which would trivially reject the positive control.

The fit tolerance is an inferred-geometry safeguard, not proof of physical surface identity: p90 plane residual2.5 cm and point residual3 cm can merge close depth layers. Keep the >=2.5 cm obstacle negative control and later independent held-out obstacle-retention denominator. No oracle geometry enters this gate. Four semantic A/B rows must classify actual consecutive native-frame context (at least the frozen three-frame history), not isolated sparse audit frames. No threshold change, production promotion or real-data bias-run clearance is granted by this read-only review yet.

Independent CPU verification at this checkpoint: all **8** current `test_return_consistency.py` tests passed in2.957 s (DELTAR Python3.8, CUDA disabled, one BLAS/two OMP threads). This includes the actual-calibration expected-rejection regression, exact legacy support-mask equivalence, unchanged oblique far tail, and selected-return slope positive control. It verifies the current conservative implementation and the diagnosed limitation; the revised owner relaxation and its plane-PASS/swap-FAIL controls are not implemented/tested at this checkpoint.

### Strict fallback and bounded four-row prepared-trial clearance

The requested low-obstacle negative control invalidated the proposed surface exception: original foreground1.04 m/rear1.065 m (25 mm separation) both pass the existing fitted-surface membership; p90 fit residual13.416 mm. A hidden rear-source substitution in neighborzone43 improves selected mean4 cm→zero while the proposed surface predicate returns no rejection. This is recorded in `evaluation/return_owner_25mm_nonidentifiability_20261115.json`. The earlier1 m layer-gap negative control was insufficient. No threshold was tuned to rescue this exception.

Current corrected-depth API explicitly rejects every `owner_policy` other than `strict`; the unsafe geometry predicate is retained only as a labeled failed-counterexample regression. Reviewer found that the strict fallback still checked only the targetzone; coder resolved this before trial. The ACTIVE path now compares exact owner IDs across the entire128×128 raster through `strict_visibility_reasons(initial,newproj)`, so neighboring owner swaps and newly/missing represented cells veto proposals too. Exact owners are deliberately conservative and still reject the legitimate actual-R/t +4 cm front-plane fixture. Do not report this strict fallback as a complete or unbiased correction method.

Final independent **11 CPU semantic tests passed in6.165 s** (CUDA disabled; DELTAR Python3.8). Both actual-calibration strict over-rejection and25mm hidden-layer nonidentifiability are explicit tests; surface dispatch raises before fitting. Reviewer independently regenerated **baseline only** for all30 real contextframes without writing datasets: every quantized supported depth is bit-identical to the saved calibrated baseline. Frozen unique indices are `[5,6,7,20,21,22,35,36,37,50,51,52,65,66,67,80,81,82,95,96,97,110,111,112,125,126,127,140,141,142]`; each target has its two true native predecessors at .06/.07 s gaps.

`run_return_variants.py` is cleared for the **bounded four-row prepared-input diagnostic only**, using the current strict-global gate. It hardcodes strict ownership, requires full150-frame source and exact30 baseline-depth equality, freezes/calculates checkpoint/raw/source/code hashes before and after, writes diagnostic/not-for-mapping outputs, and invokes no classifier, SDK, renderer, simulator step, mapping or export. Coordinator must merge the30 depth replacements into copies of all150 frames and run the unchanged classifier chronologically; sparse classification is prohibited. Then use frozen floor denominators/common-mask scoring. Zero accepted bias proposals is a valid conservative outcome, not evidence of successful correction. No production/default, tracking, floor-map or navigation acceptance follows.

Reviewed hashes at clearance: diagnostic `5719a2dcc179cea55e9b68987dc7f2050856010b5580d63f54447f854f634121`; runner `668f531f04a2d222e577d8ceb9ad05f2aa5f1cfae1934635b75d9d639fcc083d`; tests `cb9fa4e61183cc79e0a91bfa5dcc105f1b339967c7e8e274938d4c5b4524463f`.

### Independent four-row semantic trial audit — negative result

Persistent report: `evaluation/clean_slam_return_variants_floor_review_20261115.json`. Independently checked every immutable input/code/checkpoint hash listed by the completed trial. All150 baseline saved arrays (depth, labels, camera planes, confirmations, source strings and ToF records) match the original calibrated sequence exactly. All rows preserve the other120 depthframes and classify150 frames chronologically; confirmed frames are69 baseline versus43 in each candidate. Both bias arms propose678 corrections and accept **zero** under strict ownership; no bias-success claim is supported. All rows preserve measurement times, odometry, mounts and K. No tracker, PLY, render or simulator step was used by this audit.

One evaluator run compared all four folders on the same frozen10frames/stride8 and original raytruth, preserving the same **10,747 baseline-valid visible-floor** denominator for each row. The three candidate rows produce identical sampled outcomes:

| Evidence | Baseline | Each of three candidate rows |
|---|---:|---:|
| Removed-floor precision |1181/1184 =99.747%|Undefined: no ground removed|
| Fixed-baseline floor recall |1181/10747 =10.989%|0/10747 =0%|
| Retained valid depths on baseline floor |10747|10503 (244 lost)|
| Physical floor labeled obstacle |1119|0, because all evaluated valid pixels are uncertain|
| Truth-scored valid uncertainty |56.733%|100%|
| Common10503 floor rays: median physical height |10.907 mm|5.481 mm|
| Common floor rays: p95 absolute physical height |67.412 mm|72.684 mm|

The report additionally fixes the baseline >=2.5cm visible nonfloor-surface denominator, so dropped obstacle depths cannot disappear from retention assessment: baseline has **13,443** such valid samples. Every candidate retains **11,059** as valid uncertain/blocked (**82.266%**), loses **2,384** depth samples (**17.734%**), labels zero obstacle-only, and removes zero as ground. Own-valid blocked retention is100%, but this alone hides the lost measured coverage. Invalid and uncertain remain unknown/blocked, never free; this is a coverage regression and total diagnostic-label failure, not a safe clean obstacle product.

Verdict: **Gate C FAIL for every candidate**. Selected-return slope validation changes only7 depthpixels versus the no-bias/legacy-slope arm across the30 generated frames and makes no difference to sampled floor/obstacle outcomes. With zero accepted bias corrections, this trial tests conservative rejection plus slope semantics; it does not measure successful selected-return correction. Stop support/threshold tuning on this evidence. No SDK/PLY/accepted map hybrid should be generated. Any supervised-tail pilot next needs an independently frozen train/held-out split and strict measured-input/truth separation before GPU work.

### Supervised-tail pilot review begun — no training clearance yet

Frozen plan reviewed: current-room simulation feasibility only; up to240 eligible old training frames,48 fresh validation frames, fresh test captured only after checkpoint/hash freeze. Exactly100 batch1 FP32 AdamW updates on431,040 tail parameters, lr1e-5/weight_decay.01/clip.1, SILog only, no AMP/augmentation/search. Ten-minute/9.5GiB reserved-memory ceilings and original-checkpoint preservation are required in the runner; unchanged support/classifier and independent held-out surface coverage remain evaluation requirements.

Initial isolated model helpers independently pass **2 CPU tests in2.319 s**. Actual tail gradient/optimizer update changes only the four named modules, leaves frozen parameters and all BN buffers unchanged, and strict checkpoint roundtrip is exact. This does not yet test the data/split/runner, which are being implemented; no GPU clearance follows.

Provenance correction sent to coordinator/coder: old `live_capture.jsonl` stores robot pose/joints only. Unless input arrays are copied from existing immutable H5, preparation via `Sensors.render(snapshot)` produces replay-rendered simulated sensor inputs at the recorded state. Describe this accurately rather than claiming original RGB/ToF bytes were recorded. A separate optical-Z label render remains legitimate supervision provided no label/world pose enters model inputs and input ToF is independently generated by the unchanged closest-return sensor simulation. Record snapshot, scene, calibration and noise provenance/hashes. Model-input invariance under adversarial separate-label replacement must be tested with the histogram RNG held fixed.

Pilot loader follow-up: compared the new `load_measurements` against existing ZJUL5 `DataLoadPreprocess`/`default_collate` on real `coffee-preview-native-20261115/inputs/00000.h5`, with identical torch seed2026. All **14 actual model-input tensors** are bit-exact, including RGB, sampled histogram, rectangles, bool validity, every scale4/8/16 patch-info tensor and zone count (dtype/shape also exact). H5 SHA256 `a2c27fad48492fc1478478b5567bdc55d587b34452252cbfddbe19af2e53cb62` unchanged. No label enters the model-input dictionary. CPU tensor comparison0.73 s; no inference/GPU run was needed to establish loader equality.

Planner has explicitly revised the split for physical access: existing48 coffee views are DEVELOPMENT validation, not unseen validation/test. Old SW training frames must exclude coffee/sofa/stairs plus spatial proximity to designated native/coffee and prospective test ray-hit surface patches. The fresh home east-facing24 s scan is reserved until checkpoint freeze. This is a sampled view-separation audit, not a guarantee from camera-position or geom-ID separation alone; large floor/wall geoms require per-surface positions. Final split/runner is still pending independent clearance.

Prospective root-owned test-capture script `/tmp/rb_capture_frozen_pilot_test.py` read before execution: bounded measured-odometry heading control, .04m/s movement, .08rad/s correction, fixed24 s phases, stale/slip/fault stops, checkpoint hash before/after, no SDK or ground-truth control. Concrete startup issue sent to root: immediate telemetry-age check can fail before the firstSTATE packet. A bounded initial telemetry grace with zero commands is appropriate; after the firstSTATE retain the existing stale limit, and require fresh valid odometry before phases/motion. Verify actual head and native cadence after capture; no retiming.

### Final supervised preparation/training feasibility clearance

Final frozen split `docs/superpowers/plans/2026-11-16-deltar-pilot-split.json` SHA256 `eda7ba504dcf231e37ca12e60a12a2a4096b461303e331857a2839a5cd734871`:161 old SW train,48 existing coffee DEVELOPMENT validation, zero test samples. Independently verified all209 exact capture/odometry line hashes (including newline), all declared whole-source hashes, valid/nonslip odometry with maximum5 ms synchronization offset, minimum train spacing.53 s, body/head/coarse-region and time-deny filters. Planner's sampled visible-surface separation reports minimum.520649 m and excludes reserved whole objects/test AABB; shared scene/materials, previously inspected highly correlated3.935 s validation and unsampled thin surfaces remain explicit limitations. This is not unseen validation/general room transfer evidence.

Final preparer `6377e719b93c71e52381a3694b49c08f4d043d67638c44b225d69a3301da8e8a` additionally verifies the manifest's EXPECTED original-source hashes before creating Sensors/output and again after rendering. Wrong expected-source hash regression rejects. Supervision files remain separate float32 optical-Z NPYs; input H5 depth is zero, validation inputs are copied byte-exact and their rendered RGB pose/visibility is checked. Aborted preparation records diagnostic failure. Cleared for root-owned preparation.

Final training runner `414e108143b82f669f2728123044d824d19bf108f28027a84ee0d93911a05ab1`, tests `a6d8fd1153d6454bc0903958eb8421451a4fd1a5cc12d5cdddd7bf10ed67f80f`: independent final **5 CPU tests passed2.305 s**. Reviewed actual100-entry seeded permutation contains100 distinct TRAIN-only identities;48 validation samples receive no gradients. Native640×480/batch1/FP32, calibrated original968-key checkpoint,431,040 named tail parameters, all BN running statistics frozen, constant AdamW lr1e-5/wd.01, existing SILog and clipping.1 match frozen literals. Optimizer is constructed after CUDA move. Frozen tensor/BN hashes, finite nonzero gradients, strict state serialization, immutable labels/inputs/code/checkpoint hashes, synchronized600 s and9.5 GiB reserved-memory ceilings and no original-checkpoint overwrite are enforced. The600 s ceiling includes both validation passes, a conservative interpretation of the budget. Actual GPU feasibility is unmeasured until root's one trial completes.

A step100 file can exist before final validation/hash checks. This is now explicit: pending, failed and completed-step100 statuses remain DIAGNOSTIC/pending EXTERNAL paired validation, and `candidate_eligible_for_fresh_test` staysfalse in every runner state. A file existing or a falling training loss cannot approve capture. Root must require complete/invariants plus the pre-score paired stop rule. This bounded runner is cleared for **ONE** root-owned100-update feasibility trial only; no source/config changes during that trial, no extra hyperparameter/checkpoint search, no production defaults or accepted map.

### Reused floor scorer extended without changing the original experiment

Root-authorized evaluator-only extension adds optional explicit `frame_indices` and `expected_frame_count`; default frozen10/150 remains unchanged. Unique integer/in-range indices are validated. Fixed baseline >=2.5cm surface-obstacle denominators now report valid blocked/obstacle-only/uncertain, invalid coverage loss and ground removal; invalid is unknown, never free. Fixed-baseline floor ground counts also explicitly require valid candidate depth. The CLI retains its original frozen10/150 behavior.

Two focused CPU regressions pass (default bounds, frozen48/60 coffee indices, duplicate/negative/noninteger/out-of-range rejection, and an invalid obstacle sample retained in the denominator). Recomputed original legacy/calibrated audit: all prior counts, metrics, raw/support errors and common-mask distributions are EXACTLY preserved. No original result files were modified.

Baseline-only coffee evidence `evaluation/clean_slam_pilot_coffee_baseline_floor_20261116.json` scores the frozen48 of the complete60-frame physical-label history: precision100%, floor recall6.3788%, uncertainty82.9518%, >=2.5cm baseline obstacle samples33,379 all valid/non-ground but only9,032 obstacle-only (27.0589%). This fails floor recall already. Candidate inference must cover all60 for chronological classification; only the48 frozen indices are scored. Root will compare trainer original48 PNGs to the preexisting baseline and use consistent inference settings for paired full60 predictions, because trainer determinism differs from the original inference wrapper default. Candidate quality has not yet been scored.

### Completed one-shot supervised pilot — rejected, no fresh test or promotion

Machine-readable decision: `evaluation/clean_slam_pilot_decision_20261116.json`, directly referencing the immutable training report, independent checkpoint/PNG audit and paired-floor report. Final appropriate combined CPU check: **14 tests passed2.889 s** across pilot preparation/training, calibrated spatial adapter and floor-scorer regressions (DELTAR Python3.8, CUDA disabled, one BLAS/two OMP threads). The old unmodified148-test suite was not rerun.

Independent checkpoint audit `evaluation/clean_slam_pilot_training_review_20261116.json` verifies all968 keys/dtypes/shapes, exactly431,040 allowed parameters in21 changed tensors, all947 frozen tensors/buffers byte-unchanged, all optimizer states at100, all100 distinct seeded train identities and exclusion of validation identities, finite recorded gradients/losses, and every declared immutable source/code/input/label/checkpoint hash. Resource envelope passes:85.723 s total trial and.771484 GiB peak reserved CUDA memory. Candidate hash `43a3d5a32795e99f3a18be22de44fc422f1768c3c31e7a9a5563bc1f99698050`; original checkpoint unchanged. Execution feasibility is established; useful adaptation is not.

Raw development-validation RMSE worsens **1.179595499→1.258412591 m**, MAE **.716864901→.778607556 m**, SILog **3.359818→3.638639**. Independent saved-PNG/label scoring confirms the regression (RMSE1.179882321→1.258705158 m; expected submillimeter quantization difference from full-precision scores). Trainer-original48 versus earlier root-baseline48 differs at3,724,163 pixels, maximum8 mm, consistent with their different deterministic-convolution settings. Do not describe those runs as byte-identical or merely1mm quantization noise. The paired physical-floor audit instead uses root baseline and candidate generated with the SAME inference script/settings; no extra GPU rerun was needed to establish the already failed raw stop rule.

Paired report `evaluation/clean_slam_pilot_coffee_paired_floor_20261116.json` independently regenerates unchanged support depths and physical-IMU+mount classifier outputs for **all60 chronological frames of both rows**, exactly reproducing every saved depth/ToF/label/plane/confirmation/source record. Input H5/checkpoint hashes pass; native maxgap.07 s; confirmed frames14→8. Frozen48 indices, stride8, original truth visibility/ranges/denominators remain unchanged.

| Paired development evidence | Original | Step100 |
|---|---:|---:|
| Removed-floor precision |2603/2603 =100%|1053/1053 =100%|
| Floor recall on fixed40,807 valid-floor denominator |6.3788%|2.5804%|
| Retained valid baseline floor samples |40,807|39,968 (839 lost)|
| Common39,968 floor rays: absolute height-error p95 |30.201 mm|35.147 mm|
| Common floor rays: signed height p95 |30.052 mm|26.934 mm|
| Truth-scored valid uncertainty |82.9518%|90.6717%|
| Fixed33,379 obstacle>=2.5cm samples still measured/non-ground |33,379|32,750 (629 lost)|
| Fixed obstacle samples labeled obstacle-only |9,032|5,480|

The favorable signed upper-tail reduction does not erase the worse absolute height tail or negative floor errors; both are reported. No observed baseline obstacle surface was removed as ground, but coverage falls1.884% and obstacle-only representation falls. Unknown/invalid remains unknown/blocked, never free. Surface-height retention is not independent low-object INSTANCE certification; no absent low-object category is declared100%.

Final verdict: **REJECT step100 and stop this pilot** under the pre-score rule (raw RMSE, floor recall, absolute height tail and coverage all fail). No fresh test capture, longer training, new guarded SDK run, hybrid PLY, production/default change, map/LC/navigation acceptance. The reusable evaluator and measured inference/training boundaries are verified; the intended clean floor-free obstacle-map product remains NOT ACCEPTED.
