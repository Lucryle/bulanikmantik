"""Downward-looking pinhole camera: ground point <-> pixel (assumption A4)."""
from __future__ import annotations

import math

import numpy as np


class DownwardCamera:
    def __init__(self, width: int, height: int, hfov_deg: float, pitch_deg: float = 90.0):
        if abs(pitch_deg - 90.0) > 1e-9:
            raise NotImplementedError("only a straight-down camera (pitch 90°) is modelled")
        self.width, self.height = int(width), int(height)
        self.xc, self.yc = width / 2.0, height / 2.0
        # focal length in pixels from the horizontal field of view
        self.f = self.xc / math.tan(math.radians(hfov_deg) / 2.0)

    def world_to_pixel(self, point_xy, uav_xy, yaw: float, altitude: float):
        """Ground point (world) -> pixel (u, v). Returns floats, may be outside the image."""
        dx, dy = point_xy[0] - uav_xy[0], point_xy[1] - uav_xy[1]
        c, s = math.cos(yaw), math.sin(yaw)
        x_b = c * dx + s * dy          # forward
        y_b = -s * dx + c * dy         # left
        u = self.xc - self.f * y_b / altitude
        v = self.yc - self.f * x_b / altitude
        return u, v

    def pixel_to_world(self, u: float, v: float, uav_xy, yaw: float, altitude: float):
        """Back-project a pixel onto the ground plane (inverse of world_to_pixel)."""
        x_b = (self.yc - v) * altitude / self.f
        y_b = (self.xc - u) * altitude / self.f
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([uav_xy[0] + c * x_b - s * y_b, uav_xy[1] + s * x_b + c * y_b])

    def in_view(self, u: float, v: float) -> bool:
        return 0.0 <= u < self.width and 0.0 <= v < self.height

    def metres_per_pixel(self, altitude: float) -> float:
        return altitude / self.f
