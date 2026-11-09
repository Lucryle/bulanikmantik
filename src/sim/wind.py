"""Wind disturbance: constant residual drift + Ornstein–Uhlenbeck gusts (A8)."""
from __future__ import annotations

import math

import numpy as np


class Wind:
    """d(t) = alpha * W * w_hat + gust(t)

    Only the fraction ``alpha`` of the wind reaches the planar model, because
    a real UAV's inner attitude controller rejects most of it. The gust is a
    2-D OU process with stationary std ``gust_sigma_factor * W``.
    """

    def __init__(self, speed: float, direction_deg: float, alpha: float,
                 gust_sigma_factor: float, gust_tau: float, rng: np.random.Generator):
        d = math.radians(direction_deg)
        self.mean = alpha * speed * np.array([math.cos(d), math.sin(d)])
        self.sigma = gust_sigma_factor * speed
        self.tau = gust_tau
        self.rng = rng
        self.gust = np.zeros(2)

    def step(self, dt: float) -> np.ndarray:
        if self.sigma > 0:
            self.gust += (-self.gust / self.tau) * dt + \
                self.sigma * math.sqrt(2.0 * dt / self.tau) * self.rng.standard_normal(2)
        return self.mean + self.gust
