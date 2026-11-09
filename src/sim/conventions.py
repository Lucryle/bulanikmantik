"""Sign conventions — defined in ONE place (assumption A17).

Frames
  * World: x east, y north, yaw ψ counter-clockwise from x (radians).
  * Body:  x forward, y left.
  * Image: u to the right, v downwards, origin top-left. The top of the
    image points forward (camera looks straight down).

Paper semantics
  * Δx = u - x_c > 0  -> target is to the RIGHT  -> turn right.
  * Δy = v - y_c < 0  -> target is AHEAD         -> move forward
    (paper p. 5: "when Δy is negative, the UAV will move forward").

Controllers output a normalised value y in [-1, 1] that grows with the
error (term order LB..RB and GB..BB). This module turns it into physical
commands.
"""
from __future__ import annotations


def yaw_rate_command(y_yaw: float, yaw_rate_max: float) -> float:
    """Normalised yaw output -> yaw rate (rad/s, CCW positive).

    y > 0 means "turn right", which is clockwise, i.e. negative yaw rate.
    """
    return -y_yaw * yaw_rate_max


def speed_command(y_speed: float, v_max: float) -> float:
    """Normalised speed output -> forward speed (m/s).

    Δy < 0 (target ahead) -> y < 0 (GB, "go") -> positive forward speed.
    """
    return -y_speed * v_max
