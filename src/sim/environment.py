"""One closed-loop tracking trial.

Every time step (dt):
  1. the camera takes an image -> simulated detector (result arrives later)
  2. delayed detections reach the particle filter -> target estimate
  3. the estimate is projected into the current image -> Δx, Δy
  4. normalise: k_x = Δx / x_c, k_y = Δy / y_c       (paper Eqs. 1–2)
  5. yaw controller(k_x), speed controller(k_y)      -> normalised outputs
  6. convert to yaw rate / forward speed             (src/sim/conventions.py)
  7. move UAV (with wind), target, and particles one step
"""
from __future__ import annotations

import time

import numpy as np

from src.control.factory import build_controller
from src.estimation.particle_filter import ParticleFilter
from src.sim.camera import DownwardCamera
from src.sim.conventions import speed_command, yaw_rate_command
from src.sim.detector import Detector
from src.sim.target import Target
from src.sim.uav import UAV
from src.sim.wind import Wind


def initial_target_position(cfg: dict, rng: np.random.Generator) -> np.ndarray:
    """Random start inside the first camera image, at least ``init_min_distance`` away.
    The UAV starts at the origin facing +x, so body frame == world frame."""
    fx, fy = cfg["target"].get("init_offset_max", [3.5, 6.0])
    d_min = cfg["target"].get("init_min_distance", 2.0)
    while True:
        p = np.array([rng.uniform(-fx, fx), rng.uniform(-fy, fy)])
        if np.hypot(*p) >= d_min:
            return p


def run_trial(cfg: dict, ctrl_cfg: dict, mf_setting: dict | None, seed: int) -> dict:
    dt, duration = cfg["sim"]["dt"], cfg["sim"]["duration"]
    n_steps = int(round(duration / dt))
    # independent random streams -> the target path / noise are identical for
    # every controller run with the same seed (paired comparison)
    rng_target, rng_det, rng_wind, rng_pf = (
        np.random.default_rng(s) for s in np.random.SeedSequence(seed).spawn(4))

    u_cfg = cfg["uav"]
    h = u_cfg["altitude"]
    uav = UAV(h, u_cfg["v_max"], u_cfg["yaw_rate_max"], u_cfg["tau_v"], u_cfg["tau_r"])
    cam = DownwardCamera(**cfg["camera"])
    target = Target(cfg["target"], initial_target_position(cfg, rng_target), rng_target)
    detector = Detector(cfg["detector"], cam, duration, rng_det)
    wind = Wind(**cfg["wind"], rng=rng_wind)
    p_cfg = cfg["particle_filter"]
    use_pf = p_cfg.get("enabled", True)
    pf = ParticleFilter(
        p_cfg["n_particles"], p_cfg["init_area"], p_cfg["process_noise_acc"],
        meas_sigma_m=p_cfg["meas_sigma_m"],
        rng=rng_pf, resample_threshold=p_cfg["resample_threshold"],
        outlier_prob=p_cfg.get("outlier_prob", 0.05), inject_frac=p_cfg.get("inject_frac", 0.05),
        jitter_pos=p_cfg.get("jitter_pos", 0.05), jitter_vel=p_cfg.get("jitter_vel", 0.2))
    yaw_ctrl = build_controller(ctrl_cfg, mf_setting, dt)
    speed_ctrl = build_controller(ctrl_cfg, mf_setting, dt)

    rec = {k: np.full((n_steps, 2), np.nan) for k in ("uav_pos", "target_pos", "est_pos")}
    rec.update({k: np.full(n_steps, np.nan) for k in
                ("t", "uav_yaw", "kx", "ky", "v_cmd", "r_cmd", "compute_ms")})
    rec["measured"] = np.zeros(n_steps, dtype=bool)

    last_raw = None
    t = 0.0
    for k in range(n_steps):
        s = uav.state
        # 1–2: perception and estimation
        detector.capture(t, target.pos, s.xy, s.yaw, h)
        detections = detector.deliver(t)
        for d in detections:
            z = cam.pixel_to_world(d.u, d.v, d.uav_xy, d.uav_yaw, h)
            if use_pf:
                pf.update(z, age=t - d.t_capture)
            else:
                last_raw = z
        if use_pf:
            est = pf.estimate()[:2] if pf.n_updates > 0 else None
        else:
            est = last_raw

        # 3–6: control
        v_cmd = r_cmd = 0.0
        if est is not None:
            u, v = cam.world_to_pixel(est, s.xy, s.yaw, h)
            kx = float(np.clip((u - cam.xc) / cam.xc, -1.0, 1.0))
            ky = float(np.clip((v - cam.yc) / cam.yc, -1.0, 1.0))
            t0 = time.perf_counter()
            y_yaw = yaw_ctrl.compute(kx)
            y_speed = speed_ctrl.compute(ky)
            rec["compute_ms"][k] = (time.perf_counter() - t0) * 1e3
            r_cmd = yaw_rate_command(y_yaw, u_cfg["yaw_rate_max"])
            v_cmd = speed_command(y_speed, u_cfg["v_max"])
            rec["kx"][k], rec["ky"][k] = kx, ky
            rec["est_pos"][k] = est

        rec["t"][k] = t
        rec["uav_pos"][k] = s.xy
        rec["uav_yaw"][k] = s.yaw
        rec["target_pos"][k] = target.pos
        rec["measured"][k] = len(detections) > 0
        rec["v_cmd"][k], rec["r_cmd"][k] = v_cmd, r_cmd

        # 7: advance the world
        uav.step(v_cmd, r_cmd, wind.step(dt), dt)
        target.step(dt)
        pf.predict(dt)
        t += dt

    rec["error"] = np.linalg.norm(rec["uav_pos"] - rec["target_pos"], axis=1)
    rec["est_error"] = np.linalg.norm(rec["est_pos"] - rec["target_pos"], axis=1)
    rec["occlusion_zones"] = cfg["detector"].get("occlusion_zones", [])
    return rec
