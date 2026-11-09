"""Type-1 and Interval Type-2 triangular membership functions.

A triangular MF is defined by three points (a, b, c): membership rises
linearly from 0 at ``a`` to 1 at ``b`` and falls back to 0 at ``c``.

An Interval Type-2 (IT2) MF is described by two type-1 MFs:
  * the upper MF (UMF), the widest/highest possible membership,
  * the lower MF (LMF), the narrowest/lowest possible membership.
The area between them is the Footprint of Uncertainty (FOU). For an input
``x`` an IT2 MF therefore returns an interval ``[lower, upper]``.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TriangularMF:
    """Type-1 triangular MF (a, b, c) with optional height and shoulders.

    ``left_shoulder`` / ``right_shoulder`` keep the membership at ``height``
    for every input left of / right of the peak ``b``. They are used for the
    outermost terms (NB, PB) so that large errors still fire a rule
    (see docs/ASSUMPTIONS.md, A19).
    """

    a: float
    b: float
    c: float
    height: float = 1.0
    left_shoulder: bool = False
    right_shoulder: bool = False

    def __post_init__(self) -> None:
        if not self.a <= self.b <= self.c:
            raise ValueError(f"need a <= b <= c, got {(self.a, self.b, self.c)}")
        if not 0.0 < self.height <= 1.0:
            raise ValueError("height must be in (0, 1]")

    def __call__(self, x):
        x = np.asarray(x, dtype=float)
        mu = np.zeros_like(x)
        if self.b > self.a:
            rising = (x > self.a) & (x < self.b)
            mu = np.where(rising, (x - self.a) / (self.b - self.a), mu)
        if self.c > self.b:
            falling = (x >= self.b) & (x < self.c)
            mu = np.where(falling, (self.c - x) / (self.c - self.b), mu)
        mu = np.where(x == self.b, 1.0, mu)
        if self.left_shoulder:
            mu = np.where(x <= self.b, 1.0, mu)
        if self.right_shoulder:
            mu = np.where(x >= self.b, 1.0, mu)
        mu = mu * self.height
        return float(mu) if mu.ndim == 0 else mu


@dataclass(frozen=True)
class IT2TriangularMF:
    """Interval Type-2 triangular MF built from an upper and a lower MF."""

    upper: TriangularMF
    lower: TriangularMF

    @classmethod
    def from_triangle(
        cls,
        a: float,
        b: float,
        c: float,
        delta: float = 0.0,
        lower_height: float = 1.0,
        left_shoulder: bool = False,
        right_shoulder: bool = False,
    ) -> "IT2TriangularMF":
        """Blur a type-1 triangle into an IT2 MF (assumption A14).

        UMF = (a - delta, b, c + delta), height 1
        LMF = (a + delta, b, c - delta), height ``lower_height``
        With ``delta = 0`` and ``lower_height = 1`` the FOU vanishes and the
        MF is an ordinary type-1 triangle.
        """
        if delta < 0:
            raise ValueError("delta must be >= 0")
        la, lc = min(a + delta, b), max(c - delta, b)
        upper = TriangularMF(a - delta, b, c + delta, 1.0, left_shoulder, right_shoulder)
        lower = TriangularMF(la, b, lc, lower_height, left_shoulder, right_shoulder)
        return cls(upper, lower)

    def __call__(self, x) -> tuple:
        """Return (lower, upper) membership of ``x``."""
        return self.lower(x), self.upper(x)


def normalize_paper_scale(points, center: float = 14.0, half_range: float = 14.0):
    """Map points from the paper's 0–28 MF scale to [-1, 1] (assumption A11)."""
    return [(p - center) / half_range for p in points]


def build_it2_terms(
    mf_setting: dict,
    delta: float,
    lower_height: float,
    center: float = 14.0,
) -> tuple[list[str], list[IT2TriangularMF]]:
    """Create the 5 normalised IT2 input terms from an ``mf_settings`` YAML dict.

    The first and last terms get shoulders (A19). Returns (names, mfs) in the
    order they appear in the file (NB, NS, MM, PS, PB).
    """
    lo, hi = mf_setting.get("scale", [0, 28])
    half_range = (hi - lo) / 2.0
    names = list(mf_setting["terms"].keys())
    mfs = []
    for i, name in enumerate(names):
        a, b, c = normalize_paper_scale(mf_setting["terms"][name], center, half_range)
        mfs.append(
            IT2TriangularMF.from_triangle(
                a, b, c, delta, lower_height,
                left_shoulder=(i == 0),
                right_shoulder=(i == len(names) - 1),
            )
        )
    return names, mfs
