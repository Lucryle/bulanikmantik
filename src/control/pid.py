"""Discrete PID baseline with output saturation and anti-windup."""
from __future__ import annotations

import numpy as np

from src.control.base import Controller


class PID(Controller):
    """u = kp*e + ki*integral(e) + kd*de/dt, clipped to [-1, 1].

    * The derivative is low-pass filtered (time constant ``d_filter_tau``)
      because the pixel error is noisy.
    * Anti-windup by conditional integration: the integrator is frozen while
      the output is saturated and the error would push it further.
    """

    def __init__(self, kp: float, ki: float, kd: float, dt: float,
                 d_filter_tau: float = 0.1, u_max: float = 1.0) -> None:
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.dt = float(dt)
        self.d_filter_tau = float(d_filter_tau)
        self.u_max = float(u_max)
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error: float | None = None
        self.d_filtered = 0.0

    def compute(self, error: float) -> float:
        e = float(error)
        if self.prev_error is not None:
            raw_d = (e - self.prev_error) / self.dt
            alpha = self.dt / (self.d_filter_tau + self.dt)
            self.d_filtered += alpha * (raw_d - self.d_filtered)
        self.prev_error = e

        u_unsat = self.kp * e + self.ki * (self.integral + e * self.dt) + self.kd * self.d_filtered
        u = float(np.clip(u_unsat, -self.u_max, self.u_max))
        saturated_same_sign = (u != u_unsat) and (np.sign(u_unsat) == np.sign(e))
        if not saturated_same_sign:
            self.integral += e * self.dt
        return u
