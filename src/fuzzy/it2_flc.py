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
