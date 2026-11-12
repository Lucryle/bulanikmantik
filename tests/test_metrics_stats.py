import numpy as np
import pytest

from src.config import load_config
from src.metrics import capture_index, trial_metrics
from src.stats import mean_ci, paired_test


def fake_record(error, dt=0.05):
    n = len(error)
    return {"t": np.arange(n) * dt, "error": np.asarray(error, float),
            "in_view": np.ones(n, bool), "compute_ms": np.full(n, 0.1),
            "v_cmd": np.zeros(n), "r_cmd": np.zeros(n)}


def test_capture_needs_continuous_hold():
    err = np.array([1, 0.4, 0.4, 1, 0.4, 0.4, 0.4, 0.4])
    assert capture_index(err, 0.5, 3) == 4        # the first short dip does not count
    assert capture_index(err, 0.5, 5) is None


def test_trial_metrics_captured():
    cfg = load_config()                            # radius 0.5 m, hold 1 s, dt 0.05 s
    err = np.concatenate([np.linspace(3, 0.6, 40), np.full(60, 0.2)])
    m = trial_metrics(fake_record(err), cfg)
    assert m["captured"] and m["capture_time"] == pytest.approx(2.0)
    assert m["tracking_error"] == pytest.approx(0.2)
    assert m["robustness"] == pytest.approx(0.0)


def test_trial_metrics_not_captured_is_censored():
    cfg = load_config()
    m = trial_metrics(fake_record(np.full(100, 2.0)), cfg)
    assert not m["captured"]
    assert m["capture_time"] == cfg["sim"]["duration"]
    assert np.isnan(m["tracking_error"])


def test_mean_ci_contains_mean():
    m, lo, hi = mean_ci([1.0, 2.0, 3.0, 4.0])
    assert m == 2.5 and lo < m < hi


def test_paired_test_detects_shift_and_drops_nan():
    rng = np.random.default_rng(0)
    b = rng.normal(5, 1, 30)
    a = b - 0.5 + rng.normal(0, 0.1, 30)
    a[0] = np.nan
    r = paired_test(a, b)
    assert r["n"] == 29
    assert r["diff"] == pytest.approx(-0.5, abs=0.1)
    assert r["p_t"] < 1e-6 and r["p_wilcoxon"] < 1e-4 and r["cohen_dz"] < -2
