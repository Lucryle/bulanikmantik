"""Run ONE closed-loop tracking trial and plot it.

Usage examples:
  python experiments/run_single_trial.py
  python experiments/run_single_trial.py --scenario C --controller pid --seed 3
  python experiments/run_single_trial.py --scenario B --mf setting2 --wind 9.6
Output: results/figures/single_trial_<scenario>_<controller>_s<seed>.png
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from src.config import ROOT, load_config, load_controller, load_mf_setting  # noqa: E402
from src.sim.environment import run_trial  # noqa: E402
from src.viz import plot_trial  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="C", choices=["A", "B", "C"])
    ap.add_argument("--controller", default="it2_paper", help="file name in configs/controllers/")
    ap.add_argument("--mf", default="benchmark", help="file name in configs/mf_settings/")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--wind", type=float, default=None, help="wind speed W in m/s")
    ap.add_argument("--no-pf", action="store_true", help="use raw detections instead of the PF")
    args = ap.parse_args()

    overrides = {"particle_filter": {"enabled": not args.no_pf}}
    if args.wind is not None:
        overrides["wind"] = {"speed": args.wind}
    cfg = load_config(args.scenario, overrides)
    result = run_trial(cfg, load_controller(args.controller), load_mf_setting(args.mf), args.seed)

    err = result["error"]
    print(f"scenario {args.scenario} | {args.controller} ({args.mf}) | seed {args.seed}")
    print(f"  mean UAV-target distance : {err.mean():.2f} m  (last 10 s: {err[result['t'] >= result['t'][-1] - 10].mean():.2f} m)")
    print(f"  mean PF estimation error : {np.nanmean(result['est_error']):.2f} m")
    print(f"  frames with a detection  : {100 * result['measured'].mean():.0f} %")
    print(f"  controller time per step : {np.nanmean(result['compute_ms']):.3f} ms")

    out = ROOT / "results" / "figures"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"single_trial_{args.scenario}_{args.controller}_s{args.seed}.png"
    plot_trial(result, str(path), title=f"Scenario {args.scenario} – {args.controller} ({args.mf}), seed {args.seed}")
    print(f"  figure: {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
