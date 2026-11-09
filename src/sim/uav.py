"""Planar UAV kinematics at constant altitude with first-order lags (A2)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class UAVState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0      # rad, CCW from world x
    v: float = 0.0        # m/s, forward speed
    r: float = 0.0        # rad/s, yaw rate

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y])


class UAV:
    def __init__(self, altitude: float, v_max: float, yaw_rate_max: float,
                 tau_v: float, tau_r: float, state: UAVState | None = None):
        self.altitude = altitude
        self.v_max, self.yaw_rate_max = v_max, yaw_rate_max
        self.tau_v, self.tau_r = tau_v, tau_r
        self.state = state or UAVState()

    def step(self, v_cmd: float, r_cmd: float, disturbance_xy, dt: float) -> UAVState:
        """Advance one time step (explicit Euler).

        v' = (v_cmd - v)/tau_v,  r' = (r_cmd - r)/tau_r,
        x' = v cos(yaw) + d_x,   y' = v sin(yaw) + d_y,  yaw' = r
        """
        s = self.state
        v_cmd = float(np.clip(v_cmd, -self.v_max, self.v_max))
        r_cmd = float(np.clip(r_cmd, -self.yaw_rate_max, self.yaw_rate_max))
        s.v += (v_cmd - s.v) * dt / self.tau_v
        s.r += (r_cmd - s.r) * dt / self.tau_r
        s.x += (s.v * math.cos(s.yaw) + disturbance_xy[0]) * dt
        s.y += (s.v * math.sin(s.yaw) + disturbance_xy[1]) * dt
        s.yaw = (s.yaw + s.r * dt + math.pi) % (2 * math.pi) - math.pi
        return s
