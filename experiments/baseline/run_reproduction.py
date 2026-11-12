"""Reproduction experiments R1–R3 (simulation counterparts of the paper's tests).

  R1  Scenarios A / B / C          : IT2-FLC (paper) vs PID
  R2  Wind 0 / 4.9 / 9.6 / 12.1 m/s : IT2-FLC (paper) vs PID, default target
  R3  MF sensitivity               : IT2-FLC with Benchmark / Setting 1 / Setting 2
  (R4, the MF and control-curve figures, is experiments/baseline/plot_control_curves.py)

Both controllers use the gains from experiments/tune.py and the same 30 test
seeds (paired comparison).

Usage:  python experiments/baseline/run_reproduction.py [--n-seeds 30]
Output: results/tables/repro_*.{csv,md}, results/figures/repro_*.png
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import pandas as pd  # noqa: E402

from src.experiment import (FIGURES, Job, paired_rows, run_jobs, save_raw, summary_table,  # noqa: E402
                            test_seeds, tuned_controller, write_table)
from src.viz import method_boxplots as boxplots  # noqa: E402

METHODS = {"PID": "pid", "IT2-FLC (paper)": "it2_paper"}
SCENARIOS = ["A", "B", "C"]
WINDS = [0.0, 4.9, 9.6, 12.1]
MF_SETTINGS = ["benchmark", "setting1", "setting2"]

# Numbers reported by the paper (docs/PAPER_VERIFICATION.md).
PAPER = {
    "abstract": {"PID": (5.2, 0.80), "FLC": (4.2, 0.68)},
    "Table 3 (benchmark)": {"PID": (4.8, 0.75), "FLC": (4.2, 0.68)},
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n-seeds", type=int, default=None)
    args = ap.parse_args()
    seeds = test_seeds(args.n_seeds)
    ctrl = {m: tuned_controller(c) for m, c in METHODS.items()}
    FIGURES.mkdir(parents=True, exist_ok=True)

    # ---- R1: scenarios ----------------------------------------------------
    jobs = [Job(sc, ctrl[m], s, tags={"method": m}) for sc in SCENARIOS for m in METHODS for s in seeds]
    r1 = run_jobs(jobs, desc="R1 scenarios")
    save_raw(r1, "repro_R1_scenarios")
    write_table(summary_table(r1, ["scenario", "method"]), "repro_R1_scenarios",
                "R1 – Scenario results (mean ± std over seeds, 95 % CI)",
                "Capture time of a trial without capture = trial length (40 s). "
                "Tracking error / robustness use captured trials only.")
    write_table(paired_rows(r1, "scenario", "IT2-FLC (paper)", "PID"), "repro_R1_stats",
                "R1 – Paired tests, IT2-FLC (paper) vs PID")
    boxplots(r1, "scenario", SCENARIOS, list(METHODS), FIGURES / "repro_R1_scenarios.png", "scenario")

    # ---- R2: wind ----------------------------------------------------------
    jobs = [Job(None, ctrl[m], s, overrides={"wind": {"speed": w}}, tags={"method": m, "wind": w})
            for w in WINDS for m in METHODS for s in seeds]
    r2 = run_jobs(jobs, desc="R2 wind")
    save_raw(r2, "repro_R2_wind")
    write_table(summary_table(r2, ["wind", "method"]), "repro_R2_wind",
                "R2 – Wind levels (default linear target, 1 m/s)",
                "Wind model: residual drift α·W + OU gusts (ASSUMPTIONS A8).")
    write_table(paired_rows(r2, "wind", "IT2-FLC (paper)", "PID"), "repro_R2_stats",
                "R2 – Paired tests, IT2-FLC (paper) vs PID")
    boxplots(r2, "wind", WINDS, list(METHODS), FIGURES / "repro_R2_wind.png", "max wind speed W (m/s)")

    # ---- R3: MF sensitivity (pooled over the three scenarios) --------------
    jobs = [Job(sc, ctrl["IT2-FLC (paper)"], s, mf=mf, tags={"method": "IT2-FLC (paper)", "mf": mf})
            for mf in MF_SETTINGS for sc in SCENARIOS for s in seeds]
    r3 = run_jobs(jobs, desc="R3 MF settings")
    pid_rows = r1[r1["method"] == "PID"].assign(mf="– (PID)")
    r3_all = pd.concat([r3, pid_rows], ignore_index=True)
    save_raw(r3_all, "repro_R3_mf")
    write_table(summary_table(r3_all, ["mf", "method"]), "repro_R3_mf_sensitivity",
                "R3 – MF sensitivity (scenarios A+B+C pooled)",
                "PID has no MFs; its row is repeated from R1 for reference.")
    boxplots(r3.assign(method="IT2-FLC (paper)"), "mf", MF_SETTINGS, ["IT2-FLC (paper)"],
             FIGURES / "repro_R3_mf_sensitivity.png", "MF setting")

    # ---- paper vs ours (relative improvement) ------------------------------
    pid = r1[r1["method"] == "PID"]
    flc = r1[r1["method"] == "IT2-FLC (paper)"]
    ours_ct = (pid["capture_time"].mean(), flc["capture_time"].mean())
    both = pid.set_index(["scenario", "seed"])["tracking_error"].notna() & \
        flc.set_index(["scenario", "seed"])["tracking_error"].notna()
    ours_te = (pid.set_index(["scenario", "seed"])["tracking_error"][both].mean(),
               flc.set_index(["scenario", "seed"])["tracking_error"][both].mean())
    rows = []
    for src_name, vals in PAPER.items():
        for i, metric in enumerate(["capture time (s)", "tracking error (m)"]):
            p_pid, p_flc = vals["PID"][i], vals["FLC"][i]
            o_pid, o_flc = (ours_ct, ours_te)[i]
            rows.append({"paper source": src_name, "metric": metric,
                         "paper PID": p_pid, "paper FLC": p_flc,
                         "paper change (%)": round(100 * (p_flc - p_pid) / p_pid, 1),
                         "our PID": round(o_pid, 3), "our IT2-FLC": round(o_flc, 3),
                         "our change (%)": round(100 * (o_flc - o_pid) / o_pid, 1)})
    write_table(pd.DataFrame(rows), "repro_vs_paper",
                "Paper vs our reproduction (relative change of the FLC versus PID)",
                "Ours: scenarios A+B+C pooled, 30 seeds each. Tracking error uses seeds where both "
                "methods captured the target. Absolute values are not comparable (simulation vs real UAV).")
    print("done: results/tables/repro_*.md, results/figures/repro_*.png")


if __name__ == "__main__":
    main()
