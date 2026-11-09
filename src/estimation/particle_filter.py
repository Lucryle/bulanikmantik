"""Particle filter for the target's ground position and velocity (A7).

Follows the six steps in the reference paper (Eqs. 18–20):
  1. initialise particles uniformly over the flight area,
  2. move every particle with the motion model,
  3. weight every particle by the measurement likelihood,
  4. normalise the weights,
  5. resample according to the weights,
  6. estimate = weighted mean of the particles.

State of one particle: [x, y, vx, vy] (world frame).
Motion model: constant velocity + random acceleration (process noise).
Measurement: the detected pixel back-projected onto the ground. Because the
detection is ``latency`` seconds old, it is compared with where each particle
WAS at capture time (x - v*latency); this compensates the delay.
A small uniform "outlier" term keeps false positives from destroying the filter.
To recover after the target was lost (sudden turn, long occlusion), a small
fraction of particles is re-drawn around every new measurement
("sensor resetting", Lenser & Veloso 2000); if the measurement is wrong, those
particles simply get low weights at the next measurements.
"""
from __future__ import annotations

import numpy as np


class ParticleFilter:
    def __init__(self, n_particles: int, init_area, process_noise_acc: float,
                 meas_sigma_m: float, rng: np.random.Generator,
                 resample_threshold: float = 0.5, outlier_prob: float = 0.05,
                 init_speed: float = 2.0, inject_frac: float = 0.05, jitter_pos: float = 0.05, jitter_vel: float = 0.2):
        self.n = int(n_particles)
        self.q = float(process_noise_acc)
        self.sigma = float(meas_sigma_m)
        self.rng = rng
        self.resample_threshold = float(resample_threshold)
        self.outlier_prob = float(outlier_prob)
        self.init_speed = float(init_speed)
        self.n_inject = int(round(inject_frac * self.n))
        self.jitter = np.array([jitter_pos, jitter_pos, jitter_vel, jitter_vel], dtype=float)
        lo, hi = init_area
        self.area = (hi - lo) ** 2
        # Step 1: uniform initialisation
        self.particles = np.column_stack([
            rng.uniform(lo, hi, self.n), rng.uniform(lo, hi, self.n),
            rng.uniform(-init_speed, init_speed, self.n), rng.uniform(-init_speed, init_speed, self.n),
        ])
        self.weights = np.full(self.n, 1.0 / self.n)
        self.n_updates = 0

    def predict(self, dt: float) -> None:
        """Step 2: constant-velocity motion with random acceleration."""
        acc = self.q * self.rng.standard_normal((self.n, 2))
        self.particles[:, 0:2] += self.particles[:, 2:4] * dt + 0.5 * acc * dt ** 2
        self.particles[:, 2:4] += acc * dt

    def update(self, z_xy, age: float = 0.0) -> None:
        """Steps 3–5 for one ground-plane measurement taken ``age`` seconds ago."""
        self._inject(z_xy, age)
        past = self.particles[:, 0:2] - self.particles[:, 2:4] * age
        d2 = np.sum((past - np.asarray(z_xy)) ** 2, axis=1)
        gauss = np.exp(-0.5 * d2 / self.sigma ** 2) / (2 * np.pi * self.sigma ** 2)
        likelihood = (1 - self.outlier_prob) * gauss + self.outlier_prob / self.area
        self.weights *= likelihood                       # step 3
        total = np.sum(self.weights)
        if total <= 0 or not np.isfinite(total):
            self.weights[:] = 1.0 / self.n
        else:
            self.weights /= total                        # step 4
        if self.effective_sample_size() < self.resample_threshold * self.n:
            self.resample()                              # step 5
        self.n_updates += 1

    def _inject(self, z_xy, age: float) -> None:
        """Replace a few random particles with ones drawn around the measurement."""
        if self.n_inject == 0:
            return
        idx = self.rng.choice(self.n, self.n_inject, replace=False)
        vel = self.rng.uniform(-self.init_speed, self.init_speed, (self.n_inject, 2))
        pos = np.asarray(z_xy) + vel * age + self.sigma * self.rng.standard_normal((self.n_inject, 2))
        self.particles[idx] = np.column_stack([pos, vel])
        self.weights[idx] = np.mean(self.weights)
        self.weights /= np.sum(self.weights)

    def effective_sample_size(self) -> float:
        return 1.0 / np.sum(self.weights ** 2)

    def resample(self) -> None:
        """Systematic resampling: one random offset, N evenly spaced pointers.

        After copying, every particle gets a small random jitter
        ("regularised" PF). Without it the copies stay identical, the filter
        loses diversity and can lock onto a wrong velocity.
        """
        positions = (self.rng.random() + np.arange(self.n)) / self.n
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        idx = np.searchsorted(cumulative, positions)
        self.particles = self.particles[idx] + self.jitter * self.rng.standard_normal((self.n, 4))
        self.weights[:] = 1.0 / self.n

    def estimate(self) -> np.ndarray:
        """Step 6: weighted mean state [x, y, vx, vy]."""
        return self.weights @ self.particles
