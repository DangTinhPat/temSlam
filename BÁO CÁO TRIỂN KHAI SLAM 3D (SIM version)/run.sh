#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
exec env -u PYTHONPATH .venv/bin/python app.py "$@"
