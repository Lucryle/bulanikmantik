"""Type reduction for centre-of-sets Interval Type-2 fuzzy systems.

Each rule i has
  * a firing interval [f_lower_i, f_upper_i]  (how strongly the rule fires),
  * a consequent centroid interval [c_left_i, c_right_i] (where its output lies).

Type reduction turns these intervals into one output interval [y_l, y_r].
The crisp output is then (y_l + y_r) / 2.
"""
from __future__ import annotations

import numpy as np


def _km_endpoint(centers, f_lower, f_upper, left: bool, max_iter: int = 100) -> float:
    """One end point of the Karnik–Mendel algorithm.

    y_l is the smallest possible weighted average: rules with small centres
    get their upper firing level, rules with large centres their lower one.
    y_r is the opposite. KM finds the switch point iteratively.
    """
    order = np.argsort(centers)
    c, fl, fu = centers[order], f_lower[order], f_upper[order]
    n = len(c)

    # Step 1: start from the average firing level.
    w = (fl + fu) / 2.0
    y = float(np.dot(c, w) / np.sum(w))

    for _ in range(max_iter):
        # Step 2: switch point k such that c[k] <= y <= c[k+1].
        k = int(np.clip(np.searchsorted(c, y, side="right") - 1, 0, n - 2))
        # Step 3: left of the switch point use one bound, right of it the other.
        idx = np.arange(n)
        if left:
            w = np.where(idx <= k, fu, fl)
        else:
            w = np.where(idx <= k, fl, fu)
        total = np.sum(w)
        if total <= 0.0:
            return y
        y_new = float(np.dot(c, w) / total)
        # Step 4: stop when the value no longer changes.
        if abs(y_new - y) < 1e-12:
            return y_new
        y = y_new
    return y


def karnik_mendel(f_lower, f_upper, c_left, c_right) -> tuple[float, float]:
    """Karnik–Mendel centre-of-sets type reduction. Returns (y_l, y_r)."""
    f_lower = np.asarray(f_lower, dtype=float)
    f_upper = np.asarray(f_upper, dtype=float)
    c_left = np.asarray(c_left, dtype=float)
    c_right = np.asarray(c_right, dtype=float)
    if np.sum(f_upper) <= 0.0:
        raise ValueError("no rule fires (all upper firing levels are 0)")
    if len(f_upper) == 1:
        return float(c_left[0]), float(c_right[0])
    y_l = _km_endpoint(c_left, f_lower, f_upper, left=True)
    y_r = _km_endpoint(c_right, f_lower, f_upper, left=False)
    return y_l, y_r


def exhaustive_type_reduction(f_lower, f_upper, c_left, c_right) -> tuple[float, float]:
    """Brute-force reference: try every switch point, keep min / max.

    Slow but obviously correct; used in the unit tests to check KM.
    """
    def endpoint(c, left):
        order = np.argsort(c)
        cs, fl, fu = c[order], np.asarray(f_lower, float)[order], np.asarray(f_upper, float)[order]
        values = []
        for k in range(len(cs) + 1):
            first, second = (fu, fl) if left else (fl, fu)
            w = np.concatenate([first[:k], second[k:]])
            if np.sum(w) > 0:
                values.append(np.dot(cs, w) / np.sum(w))
        return min(values) if left else max(values)

    return (
        float(endpoint(np.asarray(c_left, float), True)),
        float(endpoint(np.asarray(c_right, float), False)),
    )


def nie_tan(f_lower, f_upper, c_left, c_right) -> float:
    """Nie–Tan closed-form type reduction (fast, non-iterative).

    Uses the middle of each firing interval and of each centroid interval.
    Returns the crisp output directly.
    """
    w = (np.asarray(f_lower, float) + np.asarray(f_upper, float)) / 2.0
    c = (np.asarray(c_left, float) + np.asarray(c_right, float)) / 2.0
    if np.sum(w) <= 0.0:
        raise ValueError("no rule fires (all firing levels are 0)")
    return float(np.dot(c, w) / np.sum(w))
