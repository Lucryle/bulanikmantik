import numpy as np
import pytest

from src.config import load_controller, load_mf_setting
from src.control.factory import build_controller
from src.control.pid import PID
from src.fuzzy.it2_flc import IT2FLC
from src.fuzzy.membership import IT2TriangularMF

E = np.linspace(-1, 1, 201)


def symmetric_flc(type_reduction="km"):
    """IT2 FLC with perfectly symmetric terms (not the paper's values)."""
    peaks = [-1.0, -0.5, 0.0, 0.5, 1.0]
    terms = [IT2TriangularMF.from_triangle(p - 0.5, p, p + 0.5, 0.05, 0.8,
                                           left_shoulder=(i == 0), right_shoulder=(i == 4))
             for i, p in enumerate(peaks)]
    centers = [[-1, -0.9], [-0.55, -0.45], [-0.05, 0.05], [0.45, 0.55], [0.9, 1.0]]
    return IT2FLC([terms], [((i,), i) for i in range(5)], centers, type_reduction)


@pytest.mark.parametrize("setting", ["benchmark", "setting1", "setting2"])
def test_paper_flc_is_monotonic_and_bounded(setting):
    flc = build_controller(load_controller("it2_paper"), load_mf_setting(setting), 0.05)
    out = np.array([flc.compute(e) for e in E])
    assert np.all(np.diff(out) >= -1e-9)
    assert np.all(np.abs(out) <= 1.0)
    assert out[0] < -0.9 and out[-1] > 0.9      # big error -> big command


@pytest.mark.parametrize("tr", ["km", "nt"])
def test_symmetric_flc_is_odd_and_zero_at_zero(tr):
    flc = symmetric_flc(tr)
    assert flc.compute(0.0) == pytest.approx(0.0, abs=1e-12)
    for e in E:
        assert flc.compute(-e) == pytest.approx(-flc.compute(e), abs=1e-9)


def test_paper_benchmark_has_offset_at_zero_error():
    """Documented finding: the paper's MM peak is at 13 (not 14) on the 0-28
    scale, so zero error does NOT give zero output (see ASSUMPTIONS A20)."""
    flc = build_controller(load_controller("it2_paper"), load_mf_setting("benchmark"), 0.05)
    assert flc.compute(0.0) > 0.05


def test_input_is_clipped():
    flc = symmetric_flc()
    assert flc.compute(5.0) == pytest.approx(flc.compute(1.0))


def test_pid_proportional_only():
    pid = PID(kp=0.5, ki=0.0, kd=0.0, dt=0.05)
    assert pid.compute(0.4) == pytest.approx(0.2)


def test_pid_saturates():
    pid = PID(kp=10.0, ki=0.0, kd=0.0, dt=0.05)
    assert pid.compute(0.5) == 1.0 and pid.compute(-0.5) == -1.0


def test_pid_anti_windup():
    """While saturated, the integrator must not keep growing."""
    pid = PID(kp=2.0, ki=1.0, kd=0.0, dt=0.05)
    for _ in range(200):
        pid.compute(1.0)
    assert pid.integral < 0.1
    # after the error flips, the output leaves saturation immediately
    assert pid.compute(-0.3) < 0.0


def test_pid_integral_removes_steady_error():
    pid = PID(kp=0.0, ki=1.0, kd=0.0, dt=0.1)
    for _ in range(10):
        u = pid.compute(0.2)
    assert u == pytest.approx(0.2)


# ---- proposed IT2 fuzzy-PD ---------------------------------------------------
from src.fuzzy.it2_flc import FuzzyPD, pd_rule_table  # noqa: E402


def test_pd_rule_table_is_diagonal():
    table = dict(pd_rule_table())
    assert len(table) == 25
    assert table[(2, 2)] == 2                 # MM error, zero rate -> ZO
    assert table[(0, 0)] == 0 and table[(4, 4)] == 4
    assert table[(3, 1)] == 2                 # positive but shrinking error -> ZO
    for (i, j), c in table.items():           # anti-symmetric
        assert table[(4 - i, 4 - j)] == 4 - c


def pd_controller(setting="setting1"):
    return build_controller(load_controller("it2_pd"), load_mf_setting(setting), 0.05)


def test_pd_is_a_fuzzy_pd():
    pd_ctrl = pd_controller()
    assert isinstance(pd_ctrl, FuzzyPD)
    flc = pd_ctrl.flc
    # setting 1 is symmetric -> zero error and zero rate give zero command
    assert flc.compute(0.0, 0.0) == pytest.approx(0.0, abs=1e-9)
    # (almost) monotonic in the error for a fixed rate and in the rate for a
    # fixed error. With 2 inputs the min t-norm + FOU can create dips of
    # ~2e-5 (KM switch points move), which are irrelevant for control.
    out_e = np.array([flc.compute(e, 0.0) for e in E])
    out_d = np.array([flc.compute(0.2, d) for d in E * 5])
    assert np.all(np.diff(out_e) >= -1e-4) and np.all(np.diff(out_d) >= -1e-4)
    # a growing error asks for more than a shrinking one
    assert flc.compute(0.3, 1.0) > flc.compute(0.3, -1.0)


def test_pd_derivative_from_error_sequence():
    pd_ctrl = pd_controller()
    first = pd_ctrl.compute(0.2)              # no derivative on the first call
    assert first == pytest.approx(pd_ctrl.flc.compute(0.2, 0.0))
    pd_ctrl.compute(0.3)
    assert pd_ctrl.d_filtered > 0             # error grew -> positive rate
    pd_ctrl.reset()
    assert pd_ctrl.prev_error is None and pd_ctrl.d_filtered == 0.0
