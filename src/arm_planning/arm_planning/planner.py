"""Joint-space interpolation and workspace validation, independent of ROS nodes."""

from dataclasses import dataclass
import math

from arm_control.poses import joint_positions
from arm_description import JOINT_NAMES
from arm_description.resources import load_yaml

from .model import RobotModel, TcpPosition


@dataclass(frozen=True)
class PlanningOptions:
    segment_duration: float = 2.5
    max_joint_step: float = 0.2
    velocity_scale: float = 0.6
    joint_limit_margin: float = 0.02

    def __post_init__(self):
        for name in ("segment_duration", "max_joint_step", "velocity_scale", "joint_limit_margin"):
            if not math.isfinite(getattr(self, name)):
                raise ValueError(f"{name} must be finite.")
        if self.segment_duration <= 0 or self.max_joint_step <= 0:
            raise ValueError("segment_duration and max_joint_step must be positive.")
        if not 0 < self.velocity_scale <= 1:
            raise ValueError("velocity_scale must be in the range (0, 1].")
        if self.joint_limit_margin < 0:
            raise ValueError("joint_limit_margin must be non-negative.")


@dataclass(frozen=True)
class WorkspaceBounds:
    frame: str
    max_tcp_radius: float
    min_tcp_height: float
    max_tcp_height: float

    def check(self, label, tcp: TcpPosition):
        if tcp.radius > self.max_tcp_radius:
            raise ValueError(
                f"{label} puts tcp radius at {tcp.radius:.3f} m, "
                f"above workspace limit {self.max_tcp_radius:.3f} m."
            )
        if tcp.z < self.min_tcp_height:
            raise ValueError(
                f"{label} puts tcp height at {tcp.z:.3f} m, "
                f"below workspace limit {self.min_tcp_height:.3f} m."
            )
        if tcp.z > self.max_tcp_height:
            raise ValueError(
                f"{label} puts tcp height at {tcp.z:.3f} m, "
                f"above workspace limit {self.max_tcp_height:.3f} m."
            )


@dataclass(frozen=True)
class Waypoint:
    positions: tuple[float, ...]
    time_from_start: float


@dataclass(frozen=True)
class SegmentSummary:
    name: str
    waypoint_count: int
    max_delta: float
    duration: float
    tcp: TcpPosition


@dataclass
class TrajectoryPlan:
    waypoints: list[Waypoint]
    segments: list[SegmentSummary]
    total_duration: float

    @property
    def summary(self):
        return (f"Planned {len(self.waypoints)} waypoint(s) across "
                f"{len(self.segments)} segment(s) in {self.total_duration:.2f}s.")


class JointSpacePlanner:
    def __init__(self, model: RobotModel, workspace: WorkspaceBounds):
        self.model = model
        self.workspace = workspace

    @classmethod
    def from_config(cls):
        data = load_yaml("arm_planning", "config/workspace_guard.yaml").get("workspace_guard", {})
        try:
            workspace = WorkspaceBounds(**data)
        except TypeError as error:
            raise ValueError(f"Invalid workspace_guard configuration: {error}") from error
        return cls(RobotModel.load(root_frame=workspace.frame), workspace)

    def _check_state(self, label, positions, margin):
        for name, position in zip(JOINT_NAMES, positions):
            limit = self.model.limits[name]
            lower, upper = limit.lower + margin, limit.upper - margin
            if not lower <= position <= upper:
                raise ValueError(
                    f"{label} violates {name} limits: {position:.3f} not in "
                    f"[{lower:.3f}, {upper:.3f}] after margin."
                )
        tcp = self.model.tcp_position(positions)
        self.workspace.check(label, tcp)
        return tcp

    def plan(self, start_positions, targets, options: PlanningOptions):
        current = joint_positions(start_positions, "Current state")
        self._check_state("Current state", current, options.joint_limit_margin)
        plan = TrajectoryPlan([], [], 0.0)

        for name, values in targets:
            target = joint_positions(values, f"Target pose '{name}'")
            tcp = self._check_state(f"Target pose '{name}'", target, options.joint_limit_margin)
            deltas = [target_value - value for value, target_value in zip(current, target)]
            max_delta = max(abs(delta) for delta in deltas)
            if max_delta < 1e-6:
                plan.segments.append(SegmentSummary(name, 0, 0.0, 0.0, tcp))
                current = target
                continue

            velocity_duration = max(
                abs(delta) / max(self.model.limits[joint].velocity * options.velocity_scale, 1e-6)
                for joint, delta in zip(JOINT_NAMES, deltas)
            )
            duration = max(options.segment_duration, velocity_duration)
            steps = max(1, math.ceil(max_delta / options.max_joint_step))
            for index in range(1, steps + 1):
                ratio = index / steps
                positions = tuple(value + delta * ratio for value, delta in zip(current, deltas))
                self.workspace.check(
                    f"Interpolated waypoint for target '{name}' at ratio {ratio:.3f}",
                    self.model.tcp_position(positions),
                )
                plan.total_duration += duration / steps
                plan.waypoints.append(Waypoint(positions, plan.total_duration))
            plan.segments.append(SegmentSummary(name, steps, max_delta, duration, tcp))
            current = target
        return plan
