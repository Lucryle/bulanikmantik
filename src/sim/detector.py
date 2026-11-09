"""Simulated target detector that replaces the Vision Transformer (A5, A6).

Each frame the detector looks at the TRUE target pixel and returns
  * nothing, if the target is outside the image, inside an occlusion zone,
    or the detection is randomly missed (p_miss);
  * a random pixel (false positive) with probability p_false_positive;
  * otherwise the true pixel + Gaussian noise (sigma_px, which can grow over
    time to model illumination change).
Detections are delivered ``latency_s`` later (edge-computing delay), together
with the UAV pose at the moment the image was taken.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from src.sim.camera import DownwardCamera


@dataclass
class Detection:
    t_capture: float
    u: float
    v: float
    uav_xy: np.ndarray
    uav_yaw: float


class Detector:
    def __init__(self, cfg: dict, camera: DownwardCamera, duration: float, rng: np.random.Generator):
        self.cam = camera
        self.rng = rng
        self.p_miss = cfg.get("p_miss", 0.0)
        self.p_fp = cfg.get("p_false_positive", 0.0)
        self.latency = cfg.get("latency_s", 0.0)
        self.occlusions = [tuple(z) for z in cfg.get("occlusion_zones", [])]
        illum = cfg.get("illumination")
        if illum:
            self.sigma0, self.sigma1 = illum["sigma_px_start"], illum["sigma_px_end"]
        else:
            self.sigma0 = self.sigma1 = cfg.get("sigma_px", 0.0)
        self.duration = duration
        self.queue: deque[Detection] = deque()

    def sigma_px(self, t: float) -> float:
        frac = min(max(t / self.duration, 0.0), 1.0) if self.duration > 0 else 0.0
        return self.sigma0 + (self.sigma1 - self.sigma0) * frac

    def occluded(self, target_xy) -> bool:
        return any(np.hypot(target_xy[0] - x, target_xy[1] - y) <= r for x, y, r in self.occlusions)

    def capture(self, t: float, target_xy, uav_xy, uav_yaw: float, altitude: float) -> None:
        """Take one image at time ``t`` and queue the (possibly empty) result."""
        if self.rng.random() < self.p_fp:
            u = self.rng.uniform(0, self.cam.width)
            v = self.rng.uniform(0, self.cam.height)
        else:
            u, v = self.cam.world_to_pixel(target_xy, uav_xy, uav_yaw, altitude)
            if not self.cam.in_view(u, v) or self.occluded(target_xy) or self.rng.random() < self.p_miss:
                return
            s = self.sigma_px(t)
            u += s * self.rng.standard_normal()
            v += s * self.rng.standard_normal()
        self.queue.append(Detection(t, u, v, np.array(uav_xy, float), uav_yaw))

    def deliver(self, t: float) -> list[Detection]:
        """Return detections whose latency has passed by time ``t``."""
        out = []
        while self.queue and self.queue[0].t_capture + self.latency <= t + 1e-9:
            out.append(self.queue.popleft())
        return out
