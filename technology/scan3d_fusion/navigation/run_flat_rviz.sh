#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../../.." && pwd)"
export RB_NAV_RVIZ_CONFIG="$root/technology/scan3d_fusion/navigation/flat_2d.rviz"
exec "$root/technology/scan3d_fusion/navigation/run_rviz_navigation.sh" \
  --mesh "$root/datasets/processed/scan3d_fusion/room-estimated-plane-preview-20261112/final_mesh.stl" \
  --grid "$root/datasets/processed/scan3d_fusion/navigation-flat-2d-20261114/grid.npz" "$@"
