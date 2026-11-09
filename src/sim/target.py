"""Ground target motion models (assumption A12).

Every model moves the target at a constant speed (drawn once from
``speed_range``) and only changes its heading:
  * linear        – straight line
  * fast_linear   – straight line with occasional gentle turns (Scenario A)
  * circular      – constant turn rate
  * sinusoidal    – zig-zag: heading oscillates around a base heading (Scenario B)
  * random_turns  – straight for ``uniform_phase_s``, then sudden turns (Scenario C)
"""
from __future__ import annotations

import math

import numpy as np


class Target:
    def __init__(self, cfg: dict, start_xy, rng: np.random.Generator):
        self.cfg = cfg
        self.rng = rng
        self.motion = cfg.get("motion", "linear")
        lo, hi = cfg.get("speed_range", [1.0, 1.0])
        self.speed = float(rng.uniform(lo, hi))
        self.heading0 = float(rng.uniform(-math.pi, math.pi))
        self.heading = self.heading0
        self.pos = np.asarray(start_xy, dtype=float).copy()
        self.t = 0.0
        self.turn_rate = 0.0
        self.turn_until = 0.0
        self.next_event = self._first_event_time()

    # -- scheduling of heading changes ------------------------------------
    def _first_event_time(self) -> float:
        if self.motion == "random_turns":
            return float(self.cfg.get("uniform_phase_s", 10.0))
        if self.motion == "fast_linear":
            return float(self.rng.uniform(4.0, 8.0))
        return math.inf

    def _handle_event(self) -> None:
        if self.motion == "random_turns":
            a_lo, a_hi = self.cfg.get("turn_angle_deg", [60.0, 150.0])
            angle = math.radians(self.rng.uniform(a_lo, a_hi)) * self.rng.choice([-1.0, 1.0])
            self.heading += angle                       # sudden direction change
            i_lo, i_hi = self.cfg.get("turn_interval_s", [2.0, 5.0])
            self.next_event = self.t + float(self.rng.uniform(i_lo, i_hi))
        elif self.motion == "fast_linear":
            angle = math.radians(self.rng.uniform(10.0, 30.0)) * self.rng.choice([-1.0, 1.0])
            self.turn_rate, self.turn_until = angle / 1.0, self.t + 1.0   # gentle: over 1 s
            self.next_event = self.t + float(self.rng.uniform(4.0, 8.0))

    # -- one step -----------------------------------------------------------
    def step(self, dt: float) -> np.ndarray:
        if self.t >= self.next_event:
            self._handle_event()
        if self.motion == "circular":
            self.heading += self.speed / self.cfg.get("radius", 8.0) * dt
        elif self.motion == "sinusoidal":
            amp = math.radians(self.cfg.get("zigzag_amplitude_deg", 45.0))
            period = self.cfg.get("zigzag_period_s", 8.0)
            self.heading = self.heading0 + amp * math.sin(2 * math.pi * self.t / period)
        elif self.motion == "fast_linear" and self.t < self.turn_until:
            self.heading += self.turn_rate * dt
        elif self.motion not in ("linear", "fast_linear", "random_turns"):
            raise ValueError(f"unknown target motion: {self.motion}")
        self.pos += self.speed * dt * np.array([math.cos(self.heading), math.sin(self.heading)])
        self.t += dt
        return self.pos
