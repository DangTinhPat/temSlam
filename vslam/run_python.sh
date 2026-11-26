#!/usr/bin/env bash
set -euo pipefail
python_bin="${CUVSLAM_PYTHON:-$HOME/.venvs/cuvslam/bin/python}"
if [[ ! -x "$python_bin" ]]; then
  echo "Install vslam/requirements.txt in the cuVSLAM environment or set CUVSLAM_PYTHON." >&2
  exit 2
fi
cuda_libraries="$(env -u PYTHONPATH "$python_bin" -c 'import pathlib, sysconfig; print(":".join(str(p) for p in pathlib.Path(sysconfig.get_paths()["purelib"]).glob("nvidia/*/lib")))')"
export LD_LIBRARY_PATH="$cuda_libraries${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec env -u PYTHONPATH "$python_bin" "$@"
