# DELTAR model bundle

This folder contains the DELTAR inference source snapshot, its upstream license,
the NYU pretrained checkpoint and the authors' demo sequence used by the toolkit.

- Source snapshot: `zju3dv/deltar`, commit `f3c517562bcb2dfb9e4bbf74965f2ee76a2dd096`.
- Model weights: `weights/nyu.pt` (downloaded pretrained NYU checkpoint).
- Example input: `data/demo/` (287 H5 frames).
- Original project instructions: [UPSTREAM_README.md](UPSTREAM_README.md).
- Source license: [LICENSE](LICENSE).

The large checkpoint and H5 demo are present in the local workspace but ignored by
Git. Recreate the Python environment with
[`../../environment/requirements-gpu.txt`](../../environment/requirements-gpu.txt).
The inference entry point and run commands are documented in the toolkit README.
