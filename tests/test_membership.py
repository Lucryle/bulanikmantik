import numpy as np
import pytest

from src.config import load_mf_setting
from src.fuzzy.membership import IT2TriangularMF, TriangularMF, build_it2_terms, normalize_paper_scale


def test_triangle_values():
    mf = TriangularMF(0.0, 1.0, 3.0)
    assert mf(-1.0) == 0.0
    assert mf(0.5) == pytest.approx(0.5)
    assert mf(1.0) == 1.0
    assert mf(2.0) == pytest.approx(0.5)
    assert mf(3.0) == 0.0


def test_triangle_vectorised():
    mf = TriangularMF(0.0, 1.0, 2.0)
    np.testing.assert_allclose(mf(np.array([0.0, 0.5, 1.0, 1.5, 2.0])), [0, 0.5, 1, 0.5, 0])


def test_shoulders():
    left = TriangularMF(-1.0, -0.5, 0.0, left_shoulder=True)
    right = TriangularMF(0.0, 0.5, 1.0, right_shoulder=True)
    assert left(-5.0) == 1.0 and left(-0.25) == pytest.approx(0.5)
    assert right(5.0) == 1.0 and right(0.25) == pytest.approx(0.5)


def test_invalid_triangle():
    with pytest.raises(ValueError):
        TriangularMF(1.0, 0.0, 2.0)


def test_it2_lower_never_above_upper():
    mf = IT2TriangularMF.from_triangle(-0.5, 0.0, 0.5, delta=0.1, lower_height=0.8)
    for x in np.linspace(-1, 1, 201):
        lo, up = mf(x)
        assert 0.0 <= lo <= up <= 1.0


def test_it2_zero_fou_is_type1():
    mf = IT2TriangularMF.from_triangle(-0.5, 0.0, 0.5, delta=0.0, lower_height=1.0)
    for x in np.linspace(-1, 1, 41):
        lo, up = mf(x)
        assert lo == pytest.approx(up)


def test_paper_scale_normalisation():
    assert normalize_paper_scale([0, 14, 28]) == [-1.0, 0.0, 1.0]


@pytest.mark.parametrize("setting", ["benchmark", "setting1", "setting2"])
def test_paper_terms_cover_input_range(setting):
    """Every input in [-1, 1] must fire at least one term (upper MF > 0)."""
    _, terms = build_it2_terms(load_mf_setting(setting), delta=0.05, lower_height=0.8)
    for x in np.linspace(-1, 1, 401):
        assert max(mf(x)[1] for mf in terms) > 0.0
