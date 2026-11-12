"""Ablation and robustness experiments E3–E6 (scenario C unless noted).

  E3  FOU width sweep δ ∈ {0, 0.05, 0.1, 0.2, 0.3} (lower MF height 0.8),
      paper IT2-FLC and proposed IT2-FPD, scenarios A+B+C pooled
  E4  Measurement noise: σ_px ∈ {0, 5, 10, 20, 40} px and p_miss ∈ {0, 0.1, 0.2, 0.4}
  E5  Particle filter on/off, and perception latency ∈ {0, 0.1, 0.2, 0.3} s
  E6  Type reduction: Karnik–Mendel vs Nie–Tan vs Type-1 — closed-loop
      metrics and a micro-benchmark of the controller computation time

Usage:  python experiments/proposed/run_ablations.py [--n-seeds 30] [--only E4 E6]
Output: results/tables/abl_*.{csv,md}, results/figures/abl_*.png
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from src.config import load_mf_setting  # noqa: E402
from src.control.factory import build_controller  # noqa: E402
from src.experiment import (FIGURES, Job, paired_rows, run_jobs, save_raw, summary_table,  # noqa: E402
                            test_seeds, tuned_controller, write_table)
from src.viz import method_boxplots, sweep_plot  # noqa: E402

METHODS = {"PID": "pid", "IT2-FLC (paper)": "it2_paper", "IT2-FPD (proposed)": "it2_pd"}
FUZZY = ["IT2-FLC (paper)", "IT2-FPD (proposed)"]
PROPOSED, PAPER = "IT2-FPD (proposed)", "IT2-FLC (paper)"
SC = "C"


def e3_fou(ctrl, seeds):
    deltas = [0.0, 0.05, 0.1, 0.2, 0.3]
    jobs = [Job(sc, {**ctrl[m], "fou_delta": d, "lower_height": 0.8}, s, tags={"method": m, "fou_delta": d})
            for d in deltas for m in FUZZY for sc in ["A", "B", "C"] for s in seeds]
    df = run_jobs(jobs, desc="E3 FOU")
    save_raw(df, "abl_E3_fou")
    write_table(summary_table(df, ["fou_delta", "method"]), "abl_E3_fou",
                "E3 – FOU width δ (lower MF height 0.8; scenarios A+B+C pooled)",
                "δ = 0 with height 0.8 still has a small FOU (only the height differs); "
                "the pure type-1 case is in ext_E2_t1_vs_t2.")
    sweep_plot(df, "fou_delta", FUZZY, FIGURES / "abl_E3_fou.png", "FOU half-width δ")


def e4_noise(ctrl, seeds):
    sigmas, misses = [0.0, 5.0, 10.0, 20.0, 40.0], [0.0, 0.1, 0.2, 0.4]
    jobs = [Job(SC, ctrl[m], s, overrides={"detector": {"sigma_px": sg}}, tags={"method": m, "sigma_px": sg})
            for sg in sigmas for m in METHODS for s in seeds]
    df = run_jobs(jobs, desc="E4 pixel noise")
    save_raw(df, "abl_E4_sigma")
    write_table(summary_table(df, ["sigma_px", "method"]), "abl_E4_sigma",
                f"E4a – Pixel noise σ_px (scenario {SC})")
    write_table(paired_rows(df, "sigma_px", PROPOSED, PAPER), "abl_E4_sigma_stats",
                "E4a – Paired tests vs pixel noise, proposed vs paper")
    sweep_plot(df, "sigma_px", list(METHODS), FIGURES / "abl_E4_sigma.png", "pixel noise σ_px (px)")

    jobs = [Job(SC, ctrl[m], s, overrides={"detector": {"p_miss": p}}, tags={"method": m, "p_miss": p})
            for p in misses for m in METHODS for s in seeds]
    df = run_jobs(jobs, desc="E4 missed detections")
    save_raw(df, "abl_E4_pmiss")
    write_table(summary_table(df, ["p_miss", "method"]), "abl_E4_pmiss",
                f"E4b – Missed-detection probability (scenario {SC})")
    write_table(paired_rows(df, "p_miss", PROPOSED, PAPER), "abl_E4_pmiss_stats",
                "E4b – Paired tests vs p_miss, proposed vs paper")
    sweep_plot(df, "p_miss", list(METHODS), FIGURES / "abl_E4_pmiss.png", "missed-detection probability")


def e5_pf_latency(ctrl, seeds):
    jobs = [Job(SC, ctrl[m], s, overrides={"particle_filter": {"enabled": pf}},
                tags={"method": m, "estimator": "particle filter" if pf else "raw detection"})
            for pf in [True, False] for m in METHODS for s in seeds]
    df = run_jobs(jobs, desc="E5 PF on/off")
    save_raw(df, "abl_E5_pf")
    write_table(summary_table(df, ["estimator", "method"]), "abl_E5_pf",
                f"E5a – Particle filter vs latest raw detection (scenario {SC})")
    method_boxplots(df, "estimator", ["particle filter", "raw detection"], list(METHODS),
                    FIGURES / "abl_E5_pf.png", "target estimate used by the controller")

    lats = [0.0, 0.1, 0.2, 0.3]
    jobs = [Job(SC, ctrl[m], s, overrides={"detector": {"latency_s": la}}, tags={"method": m, "latency_s": la})
            for la in lats for m in METHODS for s in seeds]
    df = run_jobs(jobs, desc="E5 latency")
    save_raw(df, "abl_E5_latency")
    write_table(summary_table(df, ["latency_s", "method"]), "abl_E5_latency",
                f"E5b – Perception latency (scenario {SC}, PF on)")
    sweep_plot(df, "latency_s", list(METHODS), FIGURES / "abl_E5_latency.png", "perception latency (s)")


def micro_benchmark(ctrl_cfg, n=5000, seed=0):
    """Mean time of one compute() call on random inputs (µs)."""
    c = build_controller(ctrl_cfg, load_mf_setting("benchmark"), 0.05)
    x = np.random.default_rng(seed).uniform(-1, 1, n)
    for v in x[:200]:
        c.compute(v)
    t0 = time.perf_counter()
    for v in x:
        c.compute(v)
    return (time.perf_counter() - t0) / n * 1e6


def e6_type_reduction(ctrl, seeds):
    variants = {}
    for m in FUZZY:
        short = "paper" if m == PAPER else "FPD"
        variants[f"{short} KM"] = ctrl[m]
        variants[f"{short} Nie–Tan"] = {**ctrl[m], "type_reduction": "nt"}
        variants[f"{short} Type-1"] = {**ctrl[m], "fou_delta": 0.0, "lower_height": 1.0}
    variants["PID"] = ctrl["PID"]
    jobs = [Job(SC, c, s, tags={"method": m}) for m, c in variants.items() for s in seeds]
    df = run_jobs(jobs, desc="E6 type reduction")
    save_raw(df, "abl_E6_type_reduction")
    table = summary_table(df, ["method"])
    table.insert(1, "micro-benchmark (µs / call)", [round(micro_benchmark(variants[m]), 1) for m in table["method"]])
    write_table(table, "abl_E6_type_reduction",
                f"E6 – Type reduction: Karnik–Mendel vs Nie–Tan vs Type-1 (scenario {SC})",
                "ms / step = yaw + speed controller in the closed loop; micro-benchmark = one "
                "compute() call on 5000 random inputs. Times depend on the machine.")
    stats = pd.concat([paired_rows(df.assign(g=SC), "g", f"{s} Nie–Tan", f"{s} KM") for s in ["paper", "FPD"]])
    write_table(stats, "abl_E6_stats", "E6 – Paired tests, Nie–Tan vs KM")

    fig, ax = plt.subplots(figsize=(8, 4))
    names = list(table["method"])
    ax.barh(names, table["micro-benchmark (µs / call)"], color=[f"C{i % 10}" for i in range(len(names))])
    ax.set_xlabel("µs per controller call")
    ax.set_title("E6 – computation time of one controller call")
    ax.grid(alpha=0.3, axis="x")
    fig.tight_layout()
    fig.savefig(FIGURES / "abl_E6_compute_time.png", dpi=130)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n-seeds", type=int, default=None)
    ap.add_argument("--only", nargs="+", default=["E3", "E4", "E5", "E6"])
    args = ap.parse_args()
    seeds = test_seeds(args.n_seeds)
    ctrl = {m: tuned_controller(c) for m, c in METHODS.items()}
    FIGURES.mkdir(parents=True, exist_ok=True)
    steps = {"E3": e3_fou, "E4": e4_noise, "E5": e5_pf_latency, "E6": e6_type_reduction}
    for key in args.only:
        steps[key](ctrl, seeds)
    print("done: results/tables/abl_*.md, results/figures/abl_*.png")


if __name__ == "__main__":
    main()
