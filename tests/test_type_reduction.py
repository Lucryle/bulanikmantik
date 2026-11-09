import numpy as np
import pytest

from src.fuzzy.type_reduction import exhaustive_type_reduction, karnik_mendel, nie_tan


def test_km_matches_exhaustive_on_random_cases():
    rng = np.random.default_rng(0)
    for _ in range(300):
        n = int(rng.integers(2, 26))
        f_upper = rng.uniform(0, 1, n)
        f_lower = f_upper * rng.uniform(0, 1, n)
        c_left = np.sort(rng.uniform(-1, 1, n))
        c_right = c_left + rng.uniform(0, 0.2, n)
        km = karnik_mendel(f_lower, f_upper, c_left, c_right)
        ref = exhaustive_type_reduction(f_lower, f_upper, c_left, c_right)
        assert km == pytest.approx(ref, abs=1e-9)


def test_km_type1_case_is_weighted_average():
    """No uncertainty (f_lower == f_upper, c_left == c_right) -> y_l == y_r."""
    f = np.array([0.2, 0.8])
    c = np.array([-1.0, 1.0])
    y_l, y_r = karnik_mendel(f, f, c, c)
    assert y_l == pytest.approx(0.6) and y_r == pytest.approx(0.6)


def test_km_known_hand_example():
    # Two rules: centres -1 and +1, firing intervals [0, 1] each.
    # y_l: rule 1 at upper (1), rule 2 at lower (0) -> -1. y_r symmetric -> +1.
    assert karnik_mendel([0, 0], [1, 1], [-1, 1], [-1, 1]) == pytest.approx((-1.0, 1.0))


def test_km_interval_contains_nie_tan():
    rng = np.random.default_rng(1)
    for _ in range(100):
        f_upper = rng.uniform(0.1, 1, 5)
        f_lower = f_upper * rng.uniform(0, 1, 5)
        c = np.sort(rng.uniform(-1, 1, 5))
        y_l, y_r = karnik_mendel(f_lower, f_upper, c, c + 0.1)
        assert y_l - 1e-9 <= nie_tan(f_lower, f_upper, c, c + 0.1) <= y_r + 1e-9


def test_no_rule_fires_raises():
    with pytest.raises(ValueError):
        karnik_mendel([0, 0], [0, 0], [0, 1], [0, 1])
