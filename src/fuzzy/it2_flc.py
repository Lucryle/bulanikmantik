"""Interval Type-2 fuzzy logic controller (centre-of-sets, KM type reduction).

Pipeline for one control step:
  1. scale the crisp input(s) and clip them to [-1, 1]
  2. fuzzification: every input term gives an interval [lower, upper]
  3. rule firing: min t-norm over the antecedents -> [f_lower, f_upper]
  4. type reduction (Karnik–Mendel or Nie–Tan) -> [y_l, y_r]
  5. defuzzification: y = (y_l + y_r) / 2, scaled by the output gain
The output is normalised to [-1, 1]; converting it to rad/s or m/s is done
by the simulation (src/sim/conventions.py).
"""
from __future__ import annotations

from typing import Sequence

import numpy as np

from src.control.base import Controller
from src.fuzzy.membership import IT2TriangularMF, build_it2_terms
from src.fuzzy.type_reduction import karnik_mendel, nie_tan


class IT2FLC(Controller):
    """Generic IT2 FLC with any number of inputs.

    Parameters
    ----------
    input_terms : one list of IT2 MFs per input.
    rules : list of ``(antecedent_term_indices, consequent_index)``.
    output_centers : list of ``[c_left, c_right]`` per output term.
    type_reduction : ``"km"`` (Karnik–Mendel) or ``"nt"`` (Nie–Tan).
    """

    def __init__(
        self,
        input_terms: Sequence[Sequence[IT2TriangularMF]],
        rules: Sequence[tuple[tuple[int, ...], int]],
        output_centers: Sequence[Sequence[float]],
        type_reduction: str = "km",
        input_gains: Sequence[float] | None = None,
        output_gain: float = 1.0,
    ) -> None:
        self.input_terms = [list(t) for t in input_terms]
        self.rules = [(tuple(a), int(c)) for a, c in rules]
        centers = np.asarray(output_centers, dtype=float)
        self.c_left, self.c_right = centers[:, 0], centers[:, 1]
        if type_reduction not in ("km", "nt"):
            raise ValueError("type_reduction must be 'km' or 'nt'")
        self.type_reduction = type_reduction
        self.input_gains = np.ones(len(self.input_terms)) if input_gains is None else np.asarray(input_gains, float)
        self.output_gain = float(output_gain)

    def firing_intervals(self, *inputs: float) -> tuple[np.ndarray, np.ndarray]:
        """Steps 1–3: return per-rule (f_lower, f_upper)."""
        if len(inputs) != len(self.input_terms):
            raise ValueError(f"expected {len(self.input_terms)} inputs, got {len(inputs)}")
        x = np.clip(np.asarray(inputs, float) * self.input_gains, -1.0, 1.0)
        memberships = [[mf(xi) for mf in terms] for xi, terms in zip(x, self.input_terms)]
        f_lower = np.empty(len(self.rules))
        f_upper = np.empty(len(self.rules))
        for r, (antecedents, _) in enumerate(self.rules):
            lows = [memberships[i][t][0] for i, t in enumerate(antecedents)]
            ups = [memberships[i][t][1] for i, t in enumerate(antecedents)]
            f_lower[r], f_upper[r] = min(lows), min(ups)
        return f_lower, f_upper

    def output_interval(self, *inputs: float) -> tuple[float, float]:
        """Steps 1–4: return the type-reduced interval [y_l, y_r]."""
        f_lower, f_upper = self.firing_intervals(*inputs)
        if np.sum(f_upper) <= 0.0:
            return 0.0, 0.0
        consequents = [c for _, c in self.rules]
        cl, cr = self.c_left[consequents], self.c_right[consequents]
        if self.type_reduction == "nt":
            y = nie_tan(f_lower, f_upper, cl, cr)
            return y, y
        return karnik_mendel(f_lower, f_upper, cl, cr)

    def compute(self, *inputs: float) -> float:
        y_l, y_r = self.output_interval(*inputs)
        return float(np.clip(self.output_gain * (y_l + y_r) / 2.0, -1.0, 1.0))


def paper_it2_flc(ctrl_cfg: dict, mf_setting: dict) -> IT2FLC:
    """Single-input, 5-rule IT2 FLC exactly as in the reference paper.

    Rule i: "IF error is term_i THEN output is term_i" (NB->LB, ..., PB->RB
    for yaw and NB->GB, ..., PB->BB for speed). The output terms are ordered
    the same way, so both controllers share one rule table.
    """
    _, terms = build_it2_terms(
        mf_setting,
        delta=ctrl_cfg.get("fou_delta", 0.0),
        lower_height=ctrl_cfg.get("lower_height", 1.0),
        center=ctrl_cfg.get("mf_center", 14.0),
    )
    rules = [((i,), i) for i in range(len(terms))]
    return IT2FLC(
        input_terms=[terms],
        rules=rules,
        output_centers=ctrl_cfg["output_centers"],
        type_reduction=ctrl_cfg.get("type_reduction", "km"),
        input_gains=[ctrl_cfg.get("input_gain", 1.0)],
        output_gain=ctrl_cfg.get("output_gain", 1.0),
    )


# ---------------------------------------------------------------------------
# Proposed extension: two inputs (error e and error derivative ė), 25 rules.
# ---------------------------------------------------------------------------
def pd_rule_table(n_terms: int = 5) -> list[tuple[tuple[int, int], int]]:
    """Mac Vicar-Whelan diagonal rule table.

    Term indices 0..4 = NB, NS, ZE/MM, PS, PB for both inputs and
    LB/GB .. RB/BB for the output. Rule (i, j) -> clip(i + j - 2, 0, 4):
    the output grows with the error AND with the speed at which the error
    grows. Example: e = PS (3) and ė = NS (1) -> ZO (2): the error is
    positive but already shrinking, so do not push further (no overshoot).
    """
    mid = n_terms // 2
    return [((i, j), int(np.clip(i + j - mid, 0, n_terms - 1)))
            for i in range(n_terms) for j in range(n_terms)]


def derivative_terms(delta: float, lower_height: float) -> list[IT2TriangularMF]:
    """Five symmetric IT2 terms for the normalised error derivative ė
    (peaks at -1, -0.5, 0, 0.5, 1; same FOU rule as the error terms)."""
    peaks = [-1.0, -0.5, 0.0, 0.5, 1.0]
    return [IT2TriangularMF.from_triangle(p - 0.5, p, p + 0.5, delta, lower_height,
                                          left_shoulder=(i == 0), right_shoulder=(i == 4))
            for i, p in enumerate(peaks)]


class FuzzyPD(Controller):
    """IT2 fuzzy-PD controller: computes ė from consecutive errors and feeds
    (e, ė) into a 2-input IT2 FLC.

    ė is low-pass filtered (time constant ``d_filter_tau``) exactly like the
    PID derivative, because the pixel error is noisy. ``de_gain`` (seconds)
    scales ė into [-1, 1] inside the FLC (input gain of the 2nd input).
    """

    def __init__(self, flc: IT2FLC, dt: float, d_filter_tau: float = 0.1) -> None:
        self.flc = flc
        self.dt = float(dt)
        self.d_filter_tau = float(d_filter_tau)
        self.reset()

    def reset(self) -> None:
        self.prev_error: float | None = None
        self.d_filtered = 0.0

    def compute(self, error: float) -> float:
        e = float(error)
        if self.prev_error is not None:
            raw_d = (e - self.prev_error) / self.dt
            alpha = self.dt / (self.d_filter_tau + self.dt)
            self.d_filtered += alpha * (raw_d - self.d_filtered)
        self.prev_error = e
        return self.flc.compute(e, self.d_filtered)


def pd_it2_flc(ctrl_cfg: dict, mf_setting: dict, dt: float) -> FuzzyPD:
    """Proposed 2-input, 25-rule IT2 fuzzy-PD controller.

    The error input uses the SAME membership functions as the paper's
    controller (so only the added ė input differs); ė uses symmetric terms.
    """
    delta = ctrl_cfg.get("fou_delta", 0.0)
    h_low = ctrl_cfg.get("lower_height", 1.0)
    _, e_terms = build_it2_terms(mf_setting, delta=delta, lower_height=h_low,
                                 center=ctrl_cfg.get("mf_center", 14.0))
    flc = IT2FLC(
        input_terms=[e_terms, derivative_terms(delta, h_low)],
        rules=pd_rule_table(len(e_terms)),
        output_centers=ctrl_cfg["output_centers"],
        type_reduction=ctrl_cfg.get("type_reduction", "km"),
        input_gains=[ctrl_cfg.get("input_gain", 1.0), ctrl_cfg.get("de_gain", 0.2)],
        output_gain=ctrl_cfg.get("output_gain", 1.0),
    )
    return FuzzyPD(flc, dt, ctrl_cfg.get("d_filter_tau", 0.1))
