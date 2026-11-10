# Vision

Camera and ToF definitions and previews live here:

- `camera.yaml`, `tof.yaml`: sensor placement and resolution/range/noise configuration.
- `camera_viewer.py`: simulator RGB, ToF 8×8, and simulated RGB-D preview.
- `tof_placement/index.html`: browser tool for choosing the ToF pose on the head.

The model generator reads both YAML files, and the simulator dashboard starts the viewer. The placement page loads the head mesh from `/model/assets/mesh_04_00.stl`; serve the repository root over HTTP when opening it in a browser.
