"""Statistics for paired comparisons (same seeds for every method).

* mean_ci      – mean and 95 % confidence interval (Student t)
* paired_test  – paired t-test, Wilcoxon signed-rank test and Cohen's d_z
                 for two methods evaluated on the same seeds
"""
from __future__ import annotations

import numpy as np
from scipy import stats


def mean_ci(x, level: float = 0.95) -> tuple[float, float, float]:
    """Return (mean, ci_low, ci_high), NaNs ignored."""
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    if len(x) == 0:
        return np.nan, np.nan, np.nan
    m = float(np.mean(x))
    if len(x) < 2:
        return m, np.nan, np.nan
    half = stats.t.ppf(0.5 + level / 2, len(x) - 1) * np.std(x, ddof=1) / np.sqrt(len(x))
    return m, m - half, m + half


def paired_test(a, b) -> dict:
    """Compare method A with method B on the same seeds (a[i] and b[i] belong
    to seed i). Pairs where either value is NaN are dropped.

    Returns the mean difference (a - b), the relative change of A versus B
    in %, the paired t-test and Wilcoxon p-values and Cohen's d_z
    (mean difference / std of the differences).
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    ok = ~(np.isnan(a) | np.isnan(b))
    a, b = a[ok], b[ok]
    out = {"n": int(len(a)), "mean_a": np.nan, "mean_b": np.nan, "diff": np.nan, "change_pct": np.nan,
           "t": np.nan, "p_t": np.nan, "p_wilcoxon": np.nan, "cohen_dz": np.nan}
    if len(a) < 2:
        return out
    d = a - b
    out.update(mean_a=float(a.mean()), mean_b=float(b.mean()), diff=float(d.mean()))
    if b.mean() != 0:
        out["change_pct"] = float(100.0 * d.mean() / b.mean())
    sd = np.std(d, ddof=1)
    if sd > 0:
        res = stats.ttest_rel(a, b)
        out.update(t=float(res.statistic), p_t=float(res.pvalue), cohen_dz=float(d.mean() / sd))
    if np.any(d != 0):
        out["p_wilcoxon"] = float(stats.wilcoxon(a, b).pvalue)
    return out
