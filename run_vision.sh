#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$root/simulation/demo/live/headless.py" --preview "$@"
