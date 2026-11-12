"""Regenerate EVERY table and figure of the project with one command.

  1. R4     membership functions and control curves
  2.        fair tuning of all controllers (skip with --skip-tuning to reuse
            configs/controllers/*_tuned.yaml)
  3. R1–R3  reproduction experiments
  4. E1–E2  proposed controller
  5. E3–E6  ablations

Usage:  python experiments/run_all.py [--skip-tuning] [--n-seeds 30]
"""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def run(script: str, *args: str) -> None:
    print(f"\n=== {script} {' '.join(args)}", flush=True)
    subprocess.run([sys.executable, str(ROOT / script), *args], check=True, cwd=ROOT)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--skip-tuning", action="store_true")
    ap.add_argument("--n-seeds", type=int, default=None)
    args = ap.parse_args()
    seeds = ["--n-seeds", str(args.n_seeds)] if args.n_seeds else []

    run("experiments/baseline/plot_control_curves.py")
    if not args.skip_tuning:
        run("experiments/tune.py")
    run("experiments/baseline/run_reproduction.py", *seeds)
    run("experiments/proposed/run_extension.py", *seeds)
    run("experiments/proposed/run_ablations.py", *seeds)
    print("\nall results are in results/tables/ and results/figures/")


if __name__ == "__main__":
    main()
