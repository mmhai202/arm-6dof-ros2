"""One source of named poses and target selection for arm command-line tools."""

import math

from arm_description import JOINT_NAMES
from arm_description.resources import load_yaml


def joint_positions(values, label="Positions") -> tuple[float, ...]:
    positions = tuple(float(value) for value in values)
    if len(positions) != len(JOINT_NAMES) or not all(math.isfinite(value) for value in positions):
        raise ValueError(f"{label} must contain exactly {len(JOINT_NAMES)} finite joint values.")
    return positions


def load_named_poses() -> dict[str, tuple[float, ...]]:
    poses = load_yaml("arm_control", "config/named_poses.yaml").get("poses", {})
    if not poses:
        raise ValueError("No poses defined in arm_control/config/named_poses.yaml.")
    return {name: joint_positions(values, f"Pose '{name}'") for name, values in poses.items()}


def add_target_arguments(parser, poses, *, positions=True):
    targets = parser.add_mutually_exclusive_group()
    if positions:
        targets.add_argument(
            "--positions", nargs=6, type=float, metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
            help="Joint positions in joint_1..joint_6 order.",
        )
    targets.add_argument("--pose", choices=sorted(poses), help="Target named pose.")
    targets.add_argument(
        "--pose-sequence", nargs="+", choices=sorted(poses), help="Ordered named poses.",
    )
    parser.add_argument("--list-poses", action="store_true", help="List available named poses.")


def resolve_targets(args, poses):
    if getattr(args, "positions", None) is not None:
        return [("custom_target", joint_positions(args.positions))]
    names = [args.pose] if args.pose is not None else args.pose_sequence
    if not names:
        raise ValueError("--positions, --pose, or --pose-sequence is required.")
    return [(name, poses[name]) for name in names]
