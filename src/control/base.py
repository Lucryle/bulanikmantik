"""Common interface shared by every controller (PID, IT2 FLC, ...)."""
from __future__ import annotations

from abc import ABC, abstractmethod


class Controller(ABC):
    """A controller maps a normalised error in [-1, 1] to a normalised
    command in [-1, 1]. Positive error must give a positive command, so all
    controllers can be swapped in the simulation without other changes."""

    @abstractmethod
    def compute(self, *inputs: float) -> float:
        """Return the normalised command for the current error."""

    def reset(self) -> None:
        """Clear internal state (integrators, filters). Stateless by default."""
