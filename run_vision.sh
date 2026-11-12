#!/usr/bin/env bash
set -euo pipefail
exec "$(cd "$(dirname "$0")" && pwd)/technology/scan3d_fusion/live/run.sh" "$@"
