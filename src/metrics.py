"""Evaluation metrics for one tracking trial (assumptions A9, A9b).

All distances are ground-plane distances between the UAV and the TRUE target.

* capture time   – first moment from which the error stays below
                   ``capture_radius`` for at least ``capture_hold`` seconds.
                   If this never happens the trial is a "capture failure" and
                   the capture time is set to the trial length (censored value).
* tracking error – mean error AFTER capture (the paper's "tracking error").
* robustness     – standard deviation of the error after capture
                   (the paper's "robustness index (standard deviation)"; lower = better).
* extra          – mean error over the whole trial, RMSE, max error after
                   capture, % of time the target is outside the image,
                   control effort, controller computation time.
"""
from __future__ import annotations

import numpy as np


def capture_index(error: np.ndarray, radius: float, hold_steps: int) -> int | None:
    """Index of the first sample that starts a run of ``hold_steps`` samples
    with ``error < radius``. ``None`` if there is no such run."""
    run = 0
    for k, inside in enumerate(error < radius):
        run = run + 1 if inside else 0
        if run >= hold_steps:
            return k - hold_steps + 1
    return None


def trial_metrics(rec: dict, cfg: dict) -> dict:
    """Compute every metric of one trial from the record returned by ``run_trial``."""
    t, err = rec["t"], rec["error"]
    dt, duration = cfg["sim"]["dt"], cfg["sim"]["duration"]
    m_cfg = cfg["metrics"]
    hold_steps = max(1, int(round(m_cfg["capture_hold"] / dt)))
    k_cap = capture_index(err, m_cfg["capture_radius"], hold_steps)

    out = {
        "captured": k_cap is not None,
        "capture_time": float(t[k_cap]) if k_cap is not None else float(duration),
        "mean_error_all": float(np.mean(err)),
        "rmse_all": float(np.sqrt(np.mean(err ** 2))),
        "out_of_view_pct": float(100.0 * (1.0 - np.mean(rec["in_view"]))),
        "compute_ms": float(np.nanmean(rec["compute_ms"])) if np.any(~np.isnan(rec["compute_ms"])) else np.nan,
    }
    if k_cap is not None:
        after = err[k_cap:]
        out.update(tracking_error=float(np.mean(after)), robustness=float(np.std(after)),
                   max_error_after=float(np.max(after)))
    else:
        out.update(tracking_error=np.nan, robustness=np.nan, max_error_after=np.nan)

    # control effort: total variation of the normalised commands
    v_max, r_max = cfg["uav"]["v_max"], cfg["uav"]["yaw_rate_max"]
    out["control_effort"] = float(np.sum(np.abs(np.diff(rec["v_cmd"]))) / v_max +
                                  np.sum(np.abs(np.diff(rec["r_cmd"]))) / r_max)
    return out
