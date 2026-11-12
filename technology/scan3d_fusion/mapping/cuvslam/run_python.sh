#!/usr/bin/env bash
set -euo pipefail
python_bin="${CUVSLAM_PYTHON:-/home/dvt/.venvs/cuvslam/bin/python}"
if [[ ! -x "$python_bin" ]]; then
  echo "Create the cuVSLAM environment using this folder's README first." >&2
  exit 2
fi
cuda_libraries="$(env -u PYTHONPATH "$python_bin" -c 'import pathlib, sysconfig; print(":".join(str(p) for p in pathlib.Path(sysconfig.get_paths()["purelib"]).glob("nvidia/*/lib")))')"
export LD_LIBRARY_PATH="$cuda_libraries${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec env -u PYTHONPATH "$python_bin" "$@"
