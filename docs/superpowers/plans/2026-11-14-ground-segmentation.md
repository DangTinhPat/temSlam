# Ground segmentation after fusion

Approved scope: preserve DELTAR + ToF depth and full reconstruction; add ground,
obstacle and uncertain labels using synchronized wheel/IMU and the moving camera
mount. Feed labels to navigation without treating unknown space as free.

- [x] Test conservative plane fitting, camera/body movement, low obstacles,
  missing depth, temporal reset and unchanged input.
- [x] Implement separate classification and retain labels in live keyframes.
- [x] Let the scan grid consume labels, preserving original ray clearing and
  blocking ambiguous endpoints. Keep legacy scans working.
- [x] Replay the existing scan into a separate output; export classified 3D
  layers and compare grids. Do not overwrite source data or use truth geometry.
- [x] Run affected tests, review and document measured results/limitations.

Verified: 106 tests (67 navigation, 31 reconstruction, 8 IPC), independent code
review and native startup/inference smoke with eight fresh labelled frames.
Replay: 334/3198 confirmed frames, depth checksum and poses unchanged. Classified
grid has 0 free/12091 occupied/15601 unknown and 0 traversable after 25cm clearance.
Implementation complete for segmentation; scan-based navigation acceptance is
still unmet. No hardware or full live tracking/meshing route claimed.
