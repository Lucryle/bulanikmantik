"""Extension experiments E1–E2 for the proposed IT2 fuzzy-PD controller.

  E1  Proposed IT2-FPD vs paper IT2-FLC vs PID: scenarios A/B/C and wind levels
  E2  Type-1 vs Type-2: the same controllers with the FOU removed
      (fou_delta = 0, lower_height = 1, same tuned gains)
Also draws the proposed controller's control surface and example trajectories.

Usage:  python experiments/proposed/run_extension.py [--n-seeds 30]
Output: results/tables/ext_*.{csv,md}, results/figures/ext_*.png
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from src.config import load_config, load_mf_setting  # noqa: E402
from src.control.factory import build_controller  # noqa: E402
from src.experiment import (FIGURES, Job, paired_rows, run_jobs, save_raw, summary_table,  # noqa: E402
                            test_seeds, tuned_controller, write_table)
from src.sim.environment import run_trial  # noqa: E402
from src.viz import method_boxplots  # noqa: E402

METHODS = {"PID": "pid", "IT2-FLC (paper)": "it2_paper", "IT2-FPD (proposed)": "it2_pd"}
SCENARIOS = ["A", "B", "C"]
WINDS = [0.0, 4.9, 9.6, 12.1]
PROPOSED, PAPER = "IT2-FPD (proposed)", "IT2-FLC (paper)"


def type1(ctrl_cfg: dict) -> dict:
    """Same controller and gains, FOU removed -> an ordinary type-1 FLC."""
    return {**ctrl_cfg, "fou_delta": 0.0, "lower_height": 1.0}


def control_surface(ctrl_cfg: dict, path: Path) -> None:
    flc = build_controller(ctrl_cfg, load_mf_setting("benchmark"), 0.05).flc
    e = np.linspace(-1, 1, 61)
    d = np.linspace(-1, 1, 61) / ctrl_cfg["de_gain"]          # physical ė range mapped to [-1, 1]
    E, D = np.meshgrid(e, d)
    Z = np.vectorize(lambda a, b: flc.compute(a, b))(E, D)
    fig = plt.figure(figsize=(12, 4.8))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    ax.plot_surface(E, D * ctrl_cfg["de_gain"], Z, cmap="viridis", linewidth=0, antialiased=True)
    ax.set_xlabel("e (normalised)")
    ax.set_ylabel("ė · de_gain")
    ax.set_zlabel("command")
    ax.set_title("IT2-FPD control surface (benchmark MFs)")
    ax2 = fig.add_subplot(1, 2, 2)
    for rate in [-1.0, -0.5, 0.0, 0.5, 1.0]:
        ax2.plot(e, [flc.compute(x, rate / ctrl_cfg["de_gain"]) for x in e], label=f"ė·de_gain = {rate:+.1f}")
    ax2.set_xlabel("e (normalised)")
    ax2.set_ylabel("command")
    ax2.set_title("slices for fixed ė")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def example_trajectories(ctrl: dict, path: Path, seed: int = 0) -> None:
    """Top view and error over time for one seed per scenario, all methods."""
    fig, axes = plt.subplots(2, 3, figsize=(16, 9))
    for col, sc in enumerate(SCENARIOS):
        cfg = load_config(sc)
        ax_top, ax_err = axes[0, col], axes[1, col]
        for i, (m, c) in enumerate(ctrl.items()):
            rec = run_trial(cfg, c, load_mf_setting("benchmark"), seed)
            if i == 0:
                ax_top.plot(*rec["target_pos"].T, "k-", lw=1.8, label="target")
                ax_top.plot(*rec["target_pos"][0], "ko")
                for x, y, r in rec["occlusion_zones"]:
                    ax_top.add_patch(plt.Circle((x, y), r, color="grey", alpha=0.3))
            ax_top.plot(*rec["uav_pos"].T, f"C{i}-", lw=1.0, label=m)
            ax_err.plot(rec["t"], rec["error"], f"C{i}-", lw=1.0, label=m)
        ax_top.set_aspect("equal")
        ax_top.set_title(f"Scenario {sc} (seed {seed}) – top view")
        ax_top.grid(alpha=0.3)
        ax_err.axhline(cfg["metrics"]["capture_radius"], color="grey", ls="--", lw=0.8)
        ax_err.set_xlabel("time (s)")
        ax_err.set_ylabel("UAV–target distance (m)")
        ax_err.grid(alpha=0.3)
    axes[0, 0].legend(fontsize=8)
    axes[1, 0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def baseline_vs_proposed(df: pd.DataFrame) -> pd.DataFrame:
    """Guideline-style table: metric / baseline / proposed / change, pooled A+B+C."""
    base, prop = df[df["method"] == PAPER].set_index(["scenario", "seed"]), \
        df[df["method"] == PROPOSED].set_index(["scenario", "seed"])
    rows = []
    for col, label in [("capture_time", "Capture time (s)"), ("capture_rate", "Capture rate (%)"),
                       ("tracking_error", "Tracking error after capture (m)"),
                       ("robustness", "Robustness index, std (m)"), ("mean_error_all", "Mean error, whole trial (m)"),
                       ("out_of_view_pct", "Target out of view (%)"), ("control_effort", "Control effort"),
                       ("compute_ms", "Execution time (ms / step)")]:
        if col == "capture_rate":
            b, p = 100 * base["captured"].mean(), 100 * prop["captured"].mean()
        else:
            both = base[col].notna() & prop[col].reindex(base.index).notna()
            b, p = base[col][both].mean(), prop[col].reindex(base.index)[both].mean()
        rows.append({"metric": label, "baseline (paper IT2-FLC)": round(b, 3), "proposed (IT2-FPD)": round(p, 3),
                     "change (%)": round(100 * (p - b) / b, 1) if b else np.nan})
    return pd.DataFrame(rows)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n-seeds", type=int, default=None)
    args = ap.parse_args()
    seeds = test_seeds(args.n_seeds)
    ctrl = {m: tuned_controller(c) for m, c in METHODS.items()}
    FIGURES.mkdir(parents=True, exist_ok=True)

    control_surface(ctrl[PROPOSED], FIGURES / "ext_pd_control_surface.png")
    example_trajectories(ctrl, FIGURES / "ext_example_trajectories.png")

    # ---- E1: scenarios + wind ---------------------------------------------
    jobs = [Job(sc, ctrl[m], s, tags={"method": m}) for sc in SCENARIOS for m in METHODS for s in seeds]
    e1 = run_jobs(jobs, desc="E1 scenarios")
    save_raw(e1, "ext_E1_scenarios")
    write_table(summary_table(e1, ["scenario", "method"]), "ext_E1_scenarios",
                "E1 – Proposed vs paper vs PID per scenario (mean ± std, 95 % CI)")
    stats = pd.concat([paired_rows(e1, "scenario", PROPOSED, PAPER), paired_rows(e1, "scenario", PROPOSED, "PID")])
    write_table(stats, "ext_E1_stats", "E1 – Paired tests of the proposed controller",
                "change = (A − B) / B; negative = A is lower (better for every metric here).")
    write_table(baseline_vs_proposed(e1), "ext_baseline_vs_proposed",
                "Baseline vs proposed (scenarios A+B+C pooled, paired seeds)",
                "Error metrics use seeds where both controllers captured the target.")
    method_boxplots(e1, "scenario", SCENARIOS, list(METHODS), FIGURES / "ext_E1_scenarios.png", "scenario")

    jobs = [Job(None, ctrl[m], s, overrides={"wind": {"speed": w}}, tags={"method": m, "wind": w})
            for w in WINDS for m in METHODS for s in seeds]
    e1w = run_jobs(jobs, desc="E1 wind")
    save_raw(e1w, "ext_E1_wind")
    write_table(summary_table(e1w, ["wind", "method"]), "ext_E1_wind", "E1 – Wind levels, all methods")
    write_table(paired_rows(e1w, "wind", PROPOSED, PAPER), "ext_E1_wind_stats",
                "E1 – Paired tests vs wind, proposed vs paper")
    method_boxplots(e1w, "wind", WINDS, list(METHODS), FIGURES / "ext_E1_wind.png", "max wind speed W (m/s)")

    # ---- E2: type-1 vs type-2 ------------------------------------------------
    variants = {"T2 paper": ctrl[PAPER], "T1 paper": type1(ctrl[PAPER]),
                "T2 FPD": ctrl[PROPOSED], "T1 FPD": type1(ctrl[PROPOSED])}
    jobs = [Job(sc, c, s, tags={"method": m}) for sc in SCENARIOS for m, c in variants.items() for s in seeds]
    e2 = run_jobs(jobs, desc="E2 T1 vs T2")
    save_raw(e2, "ext_E2_t1_t2")
    write_table(summary_table(e2, ["scenario", "method"]), "ext_E2_t1_vs_t2",
                "E2 – Type-1 vs Type-2 (same rules, MFs and gains; FOU removed for T1)")
    stats = pd.concat([paired_rows(e2, "scenario", "T2 paper", "T1 paper"),
                       paired_rows(e2, "scenario", "T2 FPD", "T1 FPD")])
    write_table(stats, "ext_E2_stats", "E2 – Paired tests, Type-2 vs Type-1")
    method_boxplots(e2, "scenario", SCENARIOS, list(variants), FIGURES / "ext_E2_t1_vs_t2.png", "scenario")
    print("done: results/tables/ext_*.md, results/figures/ext_*.png")


if __name__ == "__main__":
    main()
