"""Reproduce every SymbioGrip software result from source, in order; stops at the first failing step.

  1. native_build.py            build test_bench and libhaptic from firmware/src + firmware/sim
  2. verify_control_theory.py   19 computed checks (filters, quantisation, passivity) -> results/verification.json
  3. test_bench                 5 SIL datasets (seed 2026, 10 seeds x modes A/B/C each)
  4. evaluator.py               statistics per dataset -> results/<dataset>/
  5. plot_benchmarks.py         figures -> results/figures/
  6. hil_trackpad_bench.py      --selftest (headless scripted trials + HTTP probe; writes nothing to data_hil)
  with --lab (v3 R&D, optional, ~2 min):
  7. arm_cycle_budget.py        static RA4M1 cycle bound of the v2 and v3 step() -> results/innovation/arm_cycles.json
  8. innovation_lab             suites ADV, S1, S2a, S2b, S3, S4, S5 -> telemetry/data_lab/ (gitignored)
  9. analyze_innovations.py     promotion rule, report -> InnovationReport.md, results/figures/fig_pareto_v3.png

Usage:  python run_pipeline.py [--skip-build] [--lab]
The firmware itself is compiled separately with arduino-cli (see README.md, "Build the firmware").
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parents[1]
DATA = REPO / "telemetry" / "data_sil"
EXE = REPO / "firmware" / "sim" / "build" / ("test_bench.exe" if sys.platform == "win32" else "test_bench")
LAB = REPO / "firmware" / "sim" / "build" / ("innovation_lab.exe" if sys.platform == "win32" else "innovation_lab")
LAB_DATA = REPO / "telemetry" / "data_lab"
LAB_SUITES = ["ADV", "S1", "S2a", "S2b", "S3", "S4", "S5"]

COMMON = ["--runs", "10", "--seed", "2026", "--modes", "ABC"]
DATASETS = [
    ("causal", ["--grip", "causal", "--schmidt-k", "0.04"]),
    ("null_grip", ["--grip", "null", "--schmidt-k", "0.04"]),
    ("sensitivity/causal_k0", ["--grip", "causal", "--schmidt-k", "0"]),
    ("sensitivity/causal_k0.08", ["--grip", "causal", "--schmidt-k", "0.08"]),
    # Dose-matched control: mode B fixed at the mean damping mode C applied in the primary dataset.
    ("sensitivity/causal_dose_matched_B8.22", ["--grip", "causal", "--schmidt-k", "0.04", "--b-fixed", "8.22"]),
]


def run(label: str, cmd: list[str]) -> None:
    t0 = time.perf_counter()
    print(f"\n=== {label}\n$ {' '.join(str(c) for c in cmd)}", flush=True)
    r = subprocess.run([str(c) for c in cmd], cwd=REPO)
    if r.returncode != 0:
        print(f"\nPIPELINE FAILED at '{label}' (exit {r.returncode})")
        sys.exit(r.returncode)
    print(f"--- ok ({time.perf_counter() - t0:.1f} s)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip-build", action="store_true")
    ap.add_argument("--lab", action="store_true", help="also run the v3 innovation lab and its analysis")
    args = ap.parse_args(argv)
    py = sys.executable
    if not args.skip_build:
        run("build native targets", [py, SCRIPTS / "native_build.py"])
    run("control-theory verification", [py, SCRIPTS / "verify_control_theory.py"])
    for name, extra in DATASETS:
        run(f"SIL dataset {name}", [EXE, "--out", DATA / name, *COMMON, *extra])
    for name, _ in DATASETS:
        run(f"evaluate {name}", [py, SCRIPTS / "evaluator.py", "--data", DATA / name])
    run("figures", [py, SCRIPTS / "plot_benchmarks.py"])
    run("desktop bench self-test", [py, SCRIPTS / "hil_trackpad_bench.py", "--selftest"])
    if args.lab:
        run("RA4M1 cycle budget", [py, SCRIPTS / "arm_cycle_budget.py"])
        for s in LAB_SUITES:
            extra = [] if s == "ADV" else ["--runs", "10", "--seed", "2026"]
            run(f"innovation lab {s}", [LAB, "--suite", s, "--out", LAB_DATA / s.lower() if s == "ADV" else LAB_DATA / s, *extra])
        run("innovation analysis", [py, SCRIPTS / "analyze_innovations.py"])
    print("\nPIPELINE PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
