#!/usr/bin/env python3
"""RB v1.3 simulator dashboard (native MuJoCo runner + CAD viewer).

Build the runner once:  cmake -S firmware -B firmware/build && cmake --build firmware/build --target rb1_sim
Run:                    python3 gui/sim_gui.py        (needs: python3-tk, mujoco)
Environment: RB1_SIM_RUNNER=<path>, RB1_SIM_MODEL=<xml>, RB1_SIM_NOISE=<scale>, RB1_SIM_ARGS="--standup-set cagain=10".
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rb1_gui.dashboard import main  # noqa: E402

if __name__ == "__main__":
    main()
