#!/usr/bin/env python3
"""Robustness scoring and evolutionary search of the RB stand-up parameters in the MuJoCo RB model.

Every candidate is a set of `--standup-set key=value` overrides of StandUp::tunedRecoveryRb (runner keys: see
sim_runner.cpp). A candidate is run through perturbed scenarios (sensor noise, wheel delay/lag/torque, CoM shift,
mass error) and scored: a stand-up that reaches Balancing at full height scores 10, a failed one a graded 0..3 by how
long Catching lasted, minus a small penalty for the nominal duration and for a body tilt-back beyond 3 deg.

    python3 tools/standup_search.py eval   params.json
    python3 tools/standup_search.py evolve params.json out.json [generations] [population] [seed]

A one-line params.json is {"cagain": 10.0, "cadamp": 1.2, ...}; `{}` scores the built-in profile.
Needs the runner:  cmake -S firmware -B firmware/build && cmake --build firmware/build --target rb1_sim
"""

from __future__ import annotations

import concurrent.futures as cf
import json
import math
from pathlib import Path
import random
import re
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "firmware/build/bin/rb1_sim"
MODEL = ROOT / "model/dynamic.xml"
WORK = Path(tempfile.mkdtemp(prefix="rb_standup_search_"))
# Arguments added to every run, e.g. RB1_SEARCH_EXTRA="--imu-model 1" for the MPU-6500 imperfections.
GLOBAL_EXTRA = __import__("os").environ.get("RB1_SEARCH_EXTRA", "").split()

# key:
SPACE = {
    "cagain": (8, 20, 1), "cadamp": (0.4, 4, 1), "hip_gain": (1, 12, 1), "hip_rate_max": (0.8, 4, 1),
    "hip_accel": (2, 20, 1), "accel_max": (4, 20, 1), "wheel_max": (10, 30, 0), "body_gain": (1, 8, 1),
    "body_rate_max": (0.8, 4, 1), "rise_rate": (0.1, 1.0, 1), "riseposture": (0.1, 1.5, 1), "sgain": (0.4, 10, 1),
    "hdead": (0.01, 0.1, 1), "khold": (0.3, 1.5, 1), "rise_accel": (0.3, 4, 1), "rise_dwell": (0.1, 1.0, 1),
    "blend": (0.03, 0.2, 1), "sint": (0.2, 8, 1), "slump": (0.1, 0.3, 0), "sintmax": (0.05, 0.3, 1),
    "slmax": (0.05, 0.3, 1), "sfilter": (0.05, 0.5, 1), "rise_hip_ff": (0.3, 1.5, 1), "assist": (0.1, 0.8, 1),
}

def variant_model(dx_mm: float = 0.0, mass_scale: float = 1.0, dz_mm: float = 0.0) -> list[str]:
    """Runner arguments for a plant with the body CoM shifted (x, z in mm) and the body mass scaled."""
    text = MODEL.read_text()
    match = re.search(r'(<body name="body_link"[^>]*>\s*<freejoint name="root"/>\s*<inertial pos=")([-0-9.e]+) ([-0-9.e]+)'
                      r' ([-0-9.e]+)(" mass=")([0-9.]+)', text)
    x, z, mass = float(match.group(2)), float(match.group(4)), float(match.group(6))
    text = (text[:match.start(2)] + f"{x + dx_mm * 1e-3:.9f} {match.group(3)} {z + dz_mm * 1e-3:.9f}"
            + match.group(5) + f"{mass * mass_scale:.4f}" + text[match.end(6):])
    path = WORK / f"var_{dx_mm:+g}_{dz_mm:+g}_{mass_scale:g}.xml"
    path.write_text(text)
    return ["MODEL", str(path)]

# Hard set: the stand-up measures the body CoM itself (StandUp probe), so the CoM cases are wide; the rest stress the
# wheel loop (delay, lag, torque), the sensors and the mass/height model.
SCENARIOS = {"nom": [], "n1": ["--sensor-noise", "1", "--noise-seed", "1"],
             "n2x": ["--sensor-noise", "2", "--noise-seed", "2"], "n3x": ["--sensor-noise", "3", "--noise-seed", "1"],
             "d70": ["--wheel-delay", "0.07"], "l120": ["--wheel-lag", "0.12"], "t10": ["--wheel-torque", "0.1"]}
SCENARIOS.update({f"com{dx:+d}": variant_model(dx_mm=dx) for dx in (-10, -6, 4, 6)})
SCENARIOS.update({f"comz{dz:+d}": variant_model(dz_mm=dz) for dz in (-10, 10)})
SCENARIOS.update({f"mass{m}": variant_model(mass_scale=m) for m in (0.8, 1.2)})
WEIGHT = {"com+6": 0.5, "n3x": 0.5}

def run_one(params: dict, extra: list[str], run_for: float = 26.0):
    cmd = [str(RUNNER), "--model", str(MODEL), "--start-key", "home", "--run-for", str(run_for), "--state-hz", "10",
           "--at", "0.3:STANDUP"]
    if extra and extra[0] == "MODEL":
        cmd[2] = extra[1]
        extra = ["--allow-model-mismatch"]
    for key, value in params.items():
        cmd += ["--standup-set", f"{key}={value}"]
    output = subprocess.run(cmd + extra + GLOBAL_EXTRA, capture_output=True, text=True).stdout
    balancing = settled = fault_t = catch_t = last = None
    tilt = 0.0     # deepest backward body pitch while catching/supported [deg, <= 0]
    for line in output.splitlines():
        if not line.startswith("STATE"):
            continue
        f = dict(kv.split("=", 1) for kv in line.split("|")[1:] if "=" in kv)
        t = float(f["t"])
        last = f
        if f.get("su_phase_name") == "Catching" and catch_t is None:
            catch_t = t
        if f.get("su_phase_name") in ("Catching", "Support") and t > 3.0:
            tilt = min(tilt, math.degrees(float(f["pitch"])))
        if f["mode"] == "Fault" and fault_t is None:
            fault_t = t
        if f["mode"] == "Balancing" and balancing is None:
            balancing = t
        if balancing is not None and settled is None and f.get("height_settled") == "1" and f.get("height_motion") == "0":
            settled = t
    ok = (last is not None and fault_t is None and last["mode"] == "Balancing" and abs(float(last["pitch"])) < 0.1
          and balancing is not None and settled is not None)
    progress = 0.0
    if not ok and last is not None:
        end = fault_t if fault_t is not None else float(last["t"])
        progress = max(0.0, min((end - (catch_t if catch_t is not None else end)) / 6.0, 1.0))
    return ok, balancing, settled, progress, tilt

def score(params: dict):
    results = {name: run_one(params, extra) for name, extra in SCENARIOS.items()}
    total = sum((10.0 if r[0] else 3.0 * r[3]) * WEIGHT.get(n, 1.0) for n, r in results.items())
    nominal = results["nom"]
    seconds = nominal[1] + 0.3 * (nominal[2] - nominal[1]) if nominal[0] else 40.0
    failed = sorted(n for n, r in results.items() if not r[0])
    tilt_penalty = 0.4 * max(0.0, -nominal[4] - 3.0) if nominal[0] else 0.0   # backward tilt beyond 3 deg during the rise
    return total - 0.5 * seconds - tilt_penalty, sum(r[0] for r in results.values()), seconds, failed

def evolve(seed: dict, out: Path, generations: int, population: int, rng_seed: int):
    rng = random.Random(rng_seed)
    best = [(score(seed), seed)]
    print("seed", best[0][0][:3], best[0][0][3], flush=True)
    for gen in range(generations):
        best.sort(key=lambda b: -b[0][0])
        parents = [b[1] for b in best[:4]]
        kids = []
        for _ in range(population):
            base = rng.choice(parents)
            kid = dict(base)
            for key, (lo, hi, _log) in SPACE.items():
                if key in kid and rng.random() < 0.35:
                    kid[key] = round(min(hi, max(lo, kid[key] * math.exp(rng.gauss(0, 0.25)))), 4)
            kids.append(kid)
        with cf.ThreadPoolExecutor(16) as pool:
            best += list(zip(pool.map(score, kids), kids))
        best.sort(key=lambda b: -b[0][0])
        print("gen", gen, [(round(b[0][0], 1), b[0][1], round(b[0][2], 1)) for b in best[:3]], best[0][0][3], flush=True)
        out.write_text(json.dumps([{"score": b[0][0], "ok": b[0][1], "seconds": b[0][2], "failed": b[0][3],
                                    "params": b[1]} for b in best[:8]], indent=1))

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    if mode == "eval" and len(sys.argv) >= 3:
        s = score(json.loads(Path(sys.argv[2]).read_text()))
        print({"score": round(s[0], 2), "scenarios_ok": f"{s[1]}/{len(SCENARIOS)}", "nominal_seconds": round(s[2], 1),
               "failed": s[3]})
    elif mode == "evolve" and len(sys.argv) >= 4:
        evolve(json.loads(Path(sys.argv[2]).read_text()), Path(sys.argv[3]),
               int(sys.argv[4]) if len(sys.argv) > 4 else 12, int(sys.argv[5]) if len(sys.argv) > 5 else 48,
               int(sys.argv[6]) if len(sys.argv) > 6 else 1)
    else:
        sys.exit(__doc__)
