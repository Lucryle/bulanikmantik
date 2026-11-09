import math

import numpy as np
import pytest

from src.config import load_config, load_controller, load_mf_setting
from src.estimation.particle_filter import ParticleFilter
from src.sim.camera import DownwardCamera
from src.sim.conventions import speed_command, yaw_rate_command
from src.sim.detector import Detector
from src.sim.environment import run_trial
from src.sim.uav import UAV
from src.sim.wind import Wind

CAM = DownwardCamera(1920, 1080, 84.0)


def test_camera_centre_and_round_trip():
    assert CAM.world_to_pixel((5.0, 3.0), (5.0, 3.0), 0.7, 10.0) == pytest.approx((960.0, 540.0))
    rng = np.random.default_rng(0)
    for _ in range(50):
        p, uav, yaw = rng.uniform(-5, 5, 2), rng.uniform(-5, 5, 2), rng.uniform(-math.pi, math.pi)
        u, v = CAM.world_to_pixel(p, uav, yaw, 10.0)
        np.testing.assert_allclose(CAM.pixel_to_world(u, v, uav, yaw, 10.0), p, atol=1e-9)


def test_sign_convention_matches_paper():
    """UAV at origin facing +x. A target to the right gives Δx > 0 and must
    make the UAV turn right; a target ahead gives Δy < 0 and must make it go
    forward (assumption A17)."""
    u, _ = CAM.world_to_pixel((0.0, -2.0), (0.0, 0.0), 0.0, 10.0)   # right of the UAV
    assert u - CAM.xc > 0
    assert yaw_rate_command(+0.5, math.pi / 3) < 0                  # clockwise = right
    _, v = CAM.world_to_pixel((2.0, 0.0), (0.0, 0.0), 0.0, 10.0)    # ahead of the UAV
    assert v - CAM.yc < 0
    assert speed_command(-0.5, 2.0) > 0                             # forward


def test_uav_reaches_commanded_speed_and_saturates():
    uav = UAV(10.0, v_max=2.0, yaw_rate_max=1.0, tau_v=0.3, tau_r=0.2)
    for _ in range(200):
        uav.step(5.0, 0.0, (0.0, 0.0), 0.05)          # command above v_max
    assert uav.state.v == pytest.approx(2.0, abs=1e-6)
    assert uav.state.y == pytest.approx(0.0) and uav.state.x > 15.0


def test_zero_wind_is_zero():
    w = Wind(0.0, 45.0, 0.04, 0.02, 2.0, np.random.default_rng(0))
    assert np.all(w.step(0.05) == 0.0)


def test_detector_latency_and_occlusion():
    det = Detector({"sigma_px": 0.0, "latency_s": 0.1, "occlusion_zones": [[10.0, 10.0, 1.0]]},
                   CAM, 10.0, np.random.default_rng(0))
    det.capture(0.0, (1.0, 0.0), (0.0, 0.0), 0.0, 10.0)
    assert det.deliver(0.05) == []                    # not yet
    assert len(det.deliver(0.1)) == 1                 # after the latency
    det.capture(0.2, (10.0, 10.0), (10.0, 10.0), 0.0, 10.0)   # inside occlusion zone
    assert det.deliver(1.0) == []


def test_particle_filter_tracks_constant_velocity_target():
    rng = np.random.default_rng(0)
    pf = ParticleFilter(500, [-20, 20], 1.0, 0.2, rng)
    p, v = np.array([3.0, -2.0]), np.array([1.5, -1.0])
    for _ in range(200):
        pf.update(p + 0.05 * rng.standard_normal(2))
        pf.predict(0.05)
        p = p + v * 0.05
    est = pf.estimate()
    assert np.linalg.norm(est[:2] - p) < 0.3
    assert np.linalg.norm(est[2:] - v) < 0.5


def test_trial_is_reproducible_and_tracks():
    cfg = load_config("C", {"sim": {"duration": 15.0}})
    args = (cfg, load_controller("it2_paper"), load_mf_setting("benchmark"), 0)
    r1, r2 = run_trial(*args), run_trial(*args)
    np.testing.assert_array_equal(r1["uav_pos"], r2["uav_pos"])
    assert r1["measured"].mean() > 0.8                # target stays in view
    assert np.nanmean(r1["est_error"]) < 0.5
