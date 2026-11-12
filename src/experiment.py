"""Helpers shared by all experiment scripts: run many trials in parallel and
write result tables.

A *job* is one trial: (scenario, controller config, MF setting, seed, config
overrides) plus free-form ``tags`` (method name, wind speed, ...) that are
copied into the result row. Every controller is run on the SAME seeds, so the
results are paired.
"""
from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd
from tqdm import tqdm

from src.config import CONFIGS, ROOT, load_config, load_controller, load_mf_setting, load_yaml
from src.metrics import trial_metrics
from src.sim.environment import run_trial
from src.stats import mean_ci, paired_test

TABLES = ROOT / "results" / "tables"
FIGURES = ROOT / "results" / "figures"
RAW = ROOT / "results" / "raw"

@dataclass
class Job:
    scenario: str | None
    ctrl_cfg: dict
    seed: int
    mf: str = "benchmark"
    overrides: dict = field(default_factory=dict)
    tags: dict = field(default_factory=dict)


def tuned_controller(name: str) -> dict:
    """Controller config with the gains found by experiments/tune.py.
    Falls back to the hand-written config if tuning has not been run."""
    path = CONFIGS / "controllers" / f"{name}_tuned.yaml"
    if path.exists():
        return load_yaml(path)
    print(f"[warning] {path.name} not found, using untuned configs/controllers/{name}.yaml")
    return load_controller(name)


def test_seeds(n: int | None = None) -> list[int]:
    s = load_config()["seeds"]
    n = s["n_test"] if n is None else n
    return list(range(s["test_start"], s["test_start"] + n))


def tuning_seeds() -> list[int]:
    return list(load_config()["seeds"]["tuning"])


def _run_job(job: Job) -> dict:
    cfg = load_config(job.scenario, job.overrides)
    rec = run_trial(cfg, job.ctrl_cfg, load_mf_setting(job.mf), job.seed)
    return {**job.tags, "scenario": job.scenario or "default", "seed": job.seed,
            **trial_metrics(rec, cfg)}


def run_jobs(jobs: list[Job], desc: str = "", workers: int | None = None) -> pd.DataFrame:
    """Run all jobs (in parallel processes) and return one row per trial."""
    workers = workers or max(1, (os.cpu_count() or 2) - 1)
    if workers == 1:
        rows = [_run_job(j) for j in tqdm(jobs, desc=desc)]
    else:
        with ProcessPoolExecutor(workers) as ex:
            rows = list(tqdm(ex.map(_run_job, jobs, chunksize=4), total=len(jobs), desc=desc))
    return pd.DataFrame(rows)


# ---------------------------------------------------------------- tables
def fmt_mean_std(x, digits: int = 2) -> str:
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if len(x) == 0:
        return "–"
    return f"{x.mean():.{digits}f} ± {x.std(ddof=1) if len(x) > 1 else 0.0:.{digits}f}"


def fmt_ci(x, digits: int = 2) -> str:
    m, lo, hi = mean_ci(x)
    if np.isnan(m):
        return "–"
    return f"{m:.{digits}f} [{lo:.{digits}f}, {hi:.{digits}f}]"


def summary_table(df: pd.DataFrame, by: list[str]) -> pd.DataFrame:
    """One row per group with the main metrics (mean ± std, and 95 % CI for
    the two headline metrics)."""
    rows = []
    for key, g in df.groupby(by, sort=False):
        key = key if isinstance(key, tuple) else (key,)
        rows.append({
            **dict(zip(by, key)),
            "n": len(g),
            "capture rate (%)": f"{100 * g['captured'].mean():.0f}",
            "capture time (s)": fmt_mean_std(g["capture_time"]),
            "capture time 95% CI": fmt_ci(g["capture_time"]),
            "tracking error (m)": fmt_mean_std(g["tracking_error"], 3),
            "tracking error 95% CI": fmt_ci(g["tracking_error"], 3),
            "robustness (std, m)": fmt_mean_std(g["robustness"], 3),
            "mean error, whole trial (m)": fmt_mean_std(g["mean_error_all"]),
            "out of view (%)": fmt_mean_std(g["out_of_view_pct"], 1),
            "control effort": fmt_mean_std(g["control_effort"], 1),
            "ms / step": fmt_mean_std(g["compute_ms"], 3),
        })
    return pd.DataFrame(rows)


def to_markdown(df: pd.DataFrame) -> str:
    cols = [str(c) for c in df.columns]
    lines = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)]
    for _, row in df.iterrows():
        cells = []
        for v in row.values:
            if isinstance(v, float):
                cells.append("–" if np.isnan(v) else f"{v:.4g}")
            else:
                cells.append(str(v))
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


def write_table(df: pd.DataFrame, name: str, title: str = "", note: str = "") -> Path:
    """Save a table as CSV and Markdown in results/tables/."""
    TABLES.mkdir(parents=True, exist_ok=True)
    df.to_csv(TABLES / f"{name}.csv", index=False)
    md = (f"# {title}\n\n" if title else "") + to_markdown(df) + (f"\n{note}\n" if note else "")
    path = TABLES / f"{name}.md"
    path.write_text(md, encoding="utf-8")
    return path


def save_raw(df: pd.DataFrame, name: str) -> None:
    """Per-trial results (git-ignored; regenerate with the scripts)."""
    RAW.mkdir(parents=True, exist_ok=True)
    df.to_csv(RAW / f"{name}.csv", index=False)


def paired_rows(df, group_col, a, b, metrics=("capture_time", "tracking_error", "mean_error_all")):
    """Paired comparison of method a vs method b inside every group."""
    rows = []
    for g, sub in df.groupby(group_col, sort=False):
        pa = sub[sub["method"] == a].set_index("seed")
        pb = sub[sub["method"] == b].set_index("seed")
        for metric in metrics:
            r = paired_test(pa[metric], pb[metric].reindex(pa.index))
            rows.append({group_col: g, "metric": metric, "A": a, "B": b, "n pairs": r["n"],
                         "mean A": r["mean_a"], "mean B": r["mean_b"], "change A vs B (%)": r["change_pct"],
                         "t": r["t"], "p (t-test)": r["p_t"], "p (Wilcoxon)": r["p_wilcoxon"],
                         "Cohen d_z": r["cohen_dz"]})
    return pd.DataFrame(rows)
