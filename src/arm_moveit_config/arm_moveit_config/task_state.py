"""Task progress shared by the physical worker and its ROS Action adapter."""

from dataclasses import dataclass


class TaskCanceled(Exception):
    """Cooperative cancellation; deliberately distinct from a planning failure."""


@dataclass(frozen=True)
class TaskProgress:
    phase: str
    stage: str
    completed: int = 0
    total: int = 0
