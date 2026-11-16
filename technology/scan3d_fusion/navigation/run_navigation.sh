#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$root"
exec env -u PYTHONPATH /home/dvt/micromamba/envs/wheel-leg-ros/bin/python \
  technology/scan3d_fusion/navigation/app.py "$@"
