#!/usr/bin/env python3

import argparse
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
import xacro
import yaml


JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def find_workspace_guard_file():
    candidates = []

    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory("arm_planning"))
                / "config"
                / "workspace_guard.yaml"
            )
        except Exception:
            pass

    candidates.append(
        Path(__file__).resolve().parents[1] / "src" / "arm_planning" / "config" / "workspace_guard.yaml"
    )

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError("Could not locate arm_planning/config/workspace_guard.yaml.")


def find_arm_xacro_file():
    candidates = []

    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory("arm_description"))
                / "urdf"
                / "arm.urdf.xacro"
            )
        except Exception:
            pass

    candidates.append(
        Path(__file__).resolve().parents[1] / "src" / "arm_description" / "urdf" / "arm.urdf.xacro"
    )

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError("Could not locate arm_description/urdf/arm.urdf.xacro.")


def find_named_poses_file():
    candidates = []

    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory("arm_control"))
                / "config"
                / "named_poses.yaml"
            )
        except Exception:
            pass

    candidates.append(
        Path(__file__).resolve().parents[1] / "src" / "arm_control" / "config" / "named_poses.yaml"
    )

    for candidate in candidates:
        if candidate.is_file():
            return candidate

    raise FileNotFoundError("Could not locate arm_control/config/named_poses.yaml.")


def load_pose_presets():
    named_poses_file = find_named_poses_file()
    with open(named_poses_file, "r", encoding="utf-8") as file_handle:
        named_poses = yaml.safe_load(file_handle) or {}

    pose_presets = named_poses.get("poses", {})
    if not pose_presets:
        raise ValueError(f"No poses defined in {named_poses_file}.")

    for pose_name, positions in pose_presets.items():
        if len(positions) != len(JOINT_NAMES):
            raise ValueError(
                f"Pose '{pose_name}' must define exactly {len(JOINT_NAMES)} joint values."
            )

    return pose_presets


POSE_PRESETS = load_pose_presets()


def load_joint_constraints():
    xacro_file = find_arm_xacro_file()
    robot_document = xacro.process_file(str(xacro_file))
    robot_root = ET.fromstring(robot_document.toxml())

    joint_constraints = {}
    for joint_element in robot_root.findall("joint"):
        joint_name = joint_element.get("name")
        if joint_name not in JOINT_NAMES:
            continue

        limit_element = joint_element.find("limit")
        if limit_element is None:
            raise ValueError(f"Joint '{joint_name}' is missing a <limit> tag in {xacro_file}.")

        joint_constraints[joint_name] = {
            "lower": float(limit_element.get("lower")),
            "upper": float(limit_element.get("upper")),
            "velocity": float(limit_element.get("velocity")),
        }

    missing_joints = [joint_name for joint_name in JOINT_NAMES if joint_name not in joint_constraints]
    if missing_joints:
        raise ValueError(f"Missing joint limits for: {', '.join(missing_joints)}")

    return joint_constraints


JOINT_CONSTRAINTS = load_joint_constraints()


def load_workspace_guard():
    workspace_guard_file = find_workspace_guard_file()
    with open(workspace_guard_file, "r", encoding="utf-8") as file_handle:
        workspace_guard = yaml.safe_load(file_handle) or {}

    workspace_guard = workspace_guard.get("workspace_guard", {})
    required_fields = ["frame", "max_tcp_radius", "min_tcp_height", "max_tcp_height"]
    missing_fields = [field_name for field_name in required_fields if field_name not in workspace_guard]
    if missing_fields:
        raise ValueError(
            f"Missing workspace guard field(s) in {workspace_guard_file}: {', '.join(missing_fields)}"
        )

    return workspace_guard


WORKSPACE_GUARD = load_workspace_guard()


def parse_xyz(text):
    return [float(value) for value in text.split()]


def rotation_matrix_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def rotation_matrix_about_axis(axis, angle):
    axis_x, axis_y, axis_z = axis
    axis_norm = math.sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z)
    if axis_norm == 0.0:
        raise ValueError("Joint axis vector must be non-zero.")

    axis_x /= axis_norm
    axis_y /= axis_norm
    axis_z /= axis_norm

    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine

    return [
        [
            cosine + axis_x * axis_x * one_minus_cosine,
            axis_x * axis_y * one_minus_cosine - axis_z * sine,
            axis_x * axis_z * one_minus_cosine + axis_y * sine,
        ],
        [
            axis_y * axis_x * one_minus_cosine + axis_z * sine,
            cosine + axis_y * axis_y * one_minus_cosine,
            axis_y * axis_z * one_minus_cosine - axis_x * sine,
        ],
        [
            axis_z * axis_x * one_minus_cosine - axis_y * sine,
            axis_z * axis_y * one_minus_cosine + axis_x * sine,
            cosine + axis_z * axis_z * one_minus_cosine,
        ],
    ]


def transform_matrix(rotation, translation):
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def identity_matrix():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def multiply_matrices(left, right):
    result = []
    for row_index in range(4):
        row = []
        for column_index in range(4):
            row.append(
                sum(left[row_index][k] * right[k][column_index] for k in range(4))
            )
        result.append(row)
    return result


def build_joint_chain_to_tcp():
    xacro_file = find_arm_xacro_file()
    robot_document = xacro.process_file(str(xacro_file))
    robot_root = ET.fromstring(robot_document.toxml())

    parent_to_joint = {}
    for joint_element in robot_root.findall("joint"):
        parent_link = joint_element.find("parent").get("link")
        parent_to_joint[parent_link] = joint_element

    current_link = "base_footprint"
    joint_chain = []

    while current_link != "tcp":
        joint_element = parent_to_joint.get(current_link)
        if joint_element is None:
            raise ValueError(f"Could not find a joint chain from {current_link} to tcp.")

        origin_element = joint_element.find("origin")
        xyz = parse_xyz(origin_element.get("xyz", "0 0 0"))
        rpy = parse_xyz(origin_element.get("rpy", "0 0 0"))
        axis_element = joint_element.find("axis")
        axis = parse_xyz(axis_element.get("xyz", "0 0 1")) if axis_element is not None else [0.0, 0.0, 1.0]

        joint_chain.append(
            {
                "name": joint_element.get("name"),
                "type": joint_element.get("type"),
                "parent": current_link,
                "child": joint_element.find("child").get("link"),
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
            }
        )
        current_link = joint_chain[-1]["child"]

    return joint_chain


JOINT_CHAIN_TO_TCP = build_joint_chain_to_tcp()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plan and optionally execute a project-owned joint-space trajectory."
    )
    parser.add_argument(
        "--positions",
        nargs=6,
        type=float,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Six joint position values in joint_1..joint_6 order.",
    )
    parser.add_argument(
        "--pose",
        choices=sorted(POSE_PRESETS.keys()),
        help="Use a predefined pose preset instead of --positions.",
    )
    parser.add_argument(
        "--pose-sequence",
        nargs="+",
        choices=sorted(POSE_PRESETS.keys()),
        help="Plan one trajectory through multiple predefined pose presets.",
    )
    parser.add_argument(
        "--list-poses",
        action="store_true",
        help="Print the available pose presets and exit.",
    )
    parser.add_argument(
        "--segment-duration",
        type=float,
        default=2.5,
        help="Seconds allocated to each target segment before interpolation.",
    )
    parser.add_argument(
        "--max-joint-step",
        type=float,
        default=0.2,
        help="Maximum joint delta per interpolated waypoint in radians.",
    )
    parser.add_argument(
        "--state-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for a complete joint state sample.",
    )
    parser.add_argument(
        "--server-timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for the action server.",
    )
    parser.add_argument(
        "--result-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the action result.",
    )
    parser.add_argument(
        "--velocity-scale",
        type=float,
        default=0.6,
        help="Scaling factor applied to each joint velocity limit when computing segment duration.",
    )
    parser.add_argument(
        "--joint-limit-margin",
        type=float,
        default=0.02,
        help="Safety margin subtracted from each joint position bound in radians.",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Build and print the trajectory summary without executing it.",
    )
    parser.add_argument(
        "--action-name",
        default="/arm_controller/follow_joint_trajectory",
        help="FollowJointTrajectory action name.",
    )
    return parser.parse_args()


def duration_from_seconds(total_seconds):
    seconds = int(total_seconds)
    nanoseconds = int(round((total_seconds - seconds) * 1_000_000_000))

    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000

    return Duration(sec=seconds, nanosec=nanoseconds)


def resolve_target_sequence(args):
    if args.positions is not None and (args.pose is not None or args.pose_sequence is not None):
        raise ValueError("--positions cannot be combined with --pose or --pose-sequence.")

    if args.pose is not None and args.pose_sequence is not None:
        raise ValueError("--pose cannot be combined with --pose-sequence.")

    if args.positions is None and args.pose is None and args.pose_sequence is None:
        raise ValueError("--positions, --pose, or --pose-sequence is required.")

    if args.positions is not None:
        return [("custom_target", list(args.positions))]

    if args.pose is not None:
        return [(args.pose, list(POSE_PRESETS[args.pose]))]

    return [(pose_name, list(POSE_PRESETS[pose_name])) for pose_name in args.pose_sequence]


def validate_positions_against_limits(label, positions, joint_limit_margin):
    for joint_name, position in zip(JOINT_NAMES, positions):
        lower = JOINT_CONSTRAINTS[joint_name]["lower"] + joint_limit_margin
        upper = JOINT_CONSTRAINTS[joint_name]["upper"] - joint_limit_margin

        if position < lower or position > upper:
            raise ValueError(
                f"{label} violates {joint_name} limits: {position:.3f} not in "
                f"[{lower:.3f}, {upper:.3f}] after margin."
            )


def compute_tcp_position(positions):
    joint_positions = {
        joint_name: position
        for joint_name, position in zip(JOINT_NAMES, positions)
    }

    transform = identity_matrix()
    for joint_spec in JOINT_CHAIN_TO_TCP:
        origin_rotation = rotation_matrix_from_rpy(*joint_spec["rpy"])
        transform = multiply_matrices(
            transform,
            transform_matrix(origin_rotation, joint_spec["xyz"]),
        )

        if joint_spec["type"] == "revolute":
            joint_angle = joint_positions[joint_spec["name"]]
            axis_rotation = rotation_matrix_about_axis(joint_spec["axis"], joint_angle)
            transform = multiply_matrices(
                transform,
                transform_matrix(axis_rotation, [0.0, 0.0, 0.0]),
            )

    return [transform[0][3], transform[1][3], transform[2][3]]


def validate_tcp_workspace(label, positions):
    tcp_x, tcp_y, tcp_z = compute_tcp_position(positions)
    tcp_radius = math.sqrt(tcp_x * tcp_x + tcp_y * tcp_y)

    max_tcp_radius = float(WORKSPACE_GUARD["max_tcp_radius"])
    min_tcp_height = float(WORKSPACE_GUARD["min_tcp_height"])
    max_tcp_height = float(WORKSPACE_GUARD["max_tcp_height"])

    if tcp_radius > max_tcp_radius:
        raise ValueError(
            f"{label} puts tcp radius at {tcp_radius:.3f} m, above workspace limit {max_tcp_radius:.3f} m."
        )

    if tcp_z < min_tcp_height:
        raise ValueError(
            f"{label} puts tcp height at {tcp_z:.3f} m, below workspace limit {min_tcp_height:.3f} m."
        )

    if tcp_z > max_tcp_height:
        raise ValueError(
            f"{label} puts tcp height at {tcp_z:.3f} m, above workspace limit {max_tcp_height:.3f} m."
        )

    return {"x": tcp_x, "y": tcp_y, "z": tcp_z, "radius": tcp_radius}


class JointStateBuffer:
    def __init__(self, node):
        self._node = node
        self._latest_positions = None
        self._subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._handle_joint_state,
            10,
        )

    def _handle_joint_state(self, message):
        joint_positions = {
            joint_name: position
            for joint_name, position in zip(message.name, message.position)
            if joint_name in JOINT_NAMES
        }

        if all(joint_name in joint_positions for joint_name in JOINT_NAMES):
            self._latest_positions = [joint_positions[joint_name] for joint_name in JOINT_NAMES]

    def wait_for_complete_state(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline and rclpy.ok():
            if self._latest_positions is not None:
                return list(self._latest_positions)
            rclpy.spin_once(self._node, timeout_sec=0.1)

        return None


def build_trajectory(
    start_positions,
    target_sequence,
    min_segment_duration,
    max_joint_step,
    velocity_scale,
    joint_limit_margin,
):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES

    current_positions = list(start_positions)
    total_duration = 0.0
    segment_summaries = []
    validate_positions_against_limits("Current state", current_positions, joint_limit_margin)
    current_tcp = validate_tcp_workspace("Current state", current_positions)

    for target_name, target_positions in target_sequence:
        validate_positions_against_limits(
            f"Target pose '{target_name}'",
            target_positions,
            joint_limit_margin,
        )
        target_tcp = validate_tcp_workspace(
            f"Target pose '{target_name}'",
            target_positions,
        )

        max_delta = max(
            abs(target - current)
            for target, current in zip(target_positions, current_positions)
        )

        if max_delta < 1e-6:
            segment_summaries.append((target_name, 0, 0.0, 0.0, target_tcp))
            current_positions = list(target_positions)
            current_tcp = target_tcp
            continue

        velocity_limited_duration = max(
            abs(target - current)
            / max(JOINT_CONSTRAINTS[joint_name]["velocity"] * velocity_scale, 1e-6)
            for joint_name, current, target in zip(JOINT_NAMES, current_positions, target_positions)
        )
        segment_duration = max(min_segment_duration, velocity_limited_duration)
        step_count = max(1, math.ceil(max_delta / max_joint_step))
        step_duration = segment_duration / step_count

        # Build a multi-point joint-space path so this mode owns interpolation itself.
        for step_index in range(1, step_count + 1):
            ratio = step_index / step_count
            interpolated_positions = [
                current + (target - current) * ratio
                for current, target in zip(current_positions, target_positions)
            ]

            total_duration += step_duration

            point = JointTrajectoryPoint()
            point.positions = interpolated_positions
            point.time_from_start = duration_from_seconds(total_duration)
            goal.trajectory.points.append(point)
            validate_tcp_workspace(
                f"Interpolated waypoint for target '{target_name}' at ratio {ratio:.3f}",
                interpolated_positions,
            )

        segment_summaries.append((target_name, step_count, max_delta, segment_duration, target_tcp))
        current_positions = list(target_positions)
        current_tcp = target_tcp

    return goal, segment_summaries, total_duration


def send_goal_and_wait(node, client, goal, server_timeout, result_timeout):
    if not client.wait_for_server(timeout_sec=server_timeout):
        node.get_logger().error("Action server is not ready.")
        return 1

    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(
        node,
        send_goal_future,
        timeout_sec=server_timeout,
    )

    if not send_goal_future.done():
        node.get_logger().error("Timed out while sending the planned trajectory.")
        return 1

    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("Planned trajectory was rejected.")
        return 1

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(
        node,
        result_future,
        timeout_sec=result_timeout,
    )

    if not result_future.done():
        node.get_logger().error("Timed out while waiting for trajectory execution.")
        return 1

    result = result_future.result()
    error_code = result.result.error_code
    error_string = result.result.error_string

    if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        node.get_logger().error(
            f"Trajectory failed. error_code={error_code}, error_string='{error_string}'"
        )
        return 1

    return 0


def main():
    args = parse_args()

    if args.list_poses:
        for pose_name in sorted(POSE_PRESETS.keys()):
            print(pose_name)
        return 0

    if args.segment_duration <= 0.0:
        print("[error] --segment-duration must be positive.", file=sys.stderr)
        return 1

    if args.max_joint_step <= 0.0:
        print("[error] --max-joint-step must be positive.", file=sys.stderr)
        return 1

    if not 0.0 < args.velocity_scale <= 1.0:
        print("[error] --velocity-scale must be in the range (0, 1].", file=sys.stderr)
        return 1

    if args.joint_limit_margin < 0.0:
        print("[error] --joint-limit-margin must be non-negative.", file=sys.stderr)
        return 1

    try:
        target_sequence = resolve_target_sequence(args)
    except ValueError as error:
        print(f"[error] {error}", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("arm_joint_space_planner")
    joint_state_buffer = JointStateBuffer(node)

    current_positions = joint_state_buffer.wait_for_complete_state(args.state_timeout)
    if current_positions is None:
        node.get_logger().error("Timed out while waiting for /joint_states.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    goal, segment_summaries, total_duration = build_trajectory(
        current_positions,
        target_sequence,
        args.segment_duration,
        args.max_joint_step,
        args.velocity_scale,
        args.joint_limit_margin,
    )

    if not goal.trajectory.points:
        node.get_logger().error("Planner produced no trajectory points.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info(
        f"Planned {len(goal.trajectory.points)} trajectory points across "
        f"{len(target_sequence)} target segment(s); total duration {total_duration:.2f}s."
    )
    for target_name, step_count, max_delta, segment_duration, tcp_summary in segment_summaries:
        node.get_logger().info(
            f"Segment '{target_name}': {step_count} point(s), max joint delta {max_delta:.3f} rad, "
            f"segment duration {segment_duration:.2f}s, tcp "
            f"(x={tcp_summary['x']:.3f}, y={tcp_summary['y']:.3f}, z={tcp_summary['z']:.3f}, "
            f"r={tcp_summary['radius']:.3f}) m."
        )

    if args.plan_only:
        node.get_logger().info("Plan-only mode enabled; skipping execution.")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    client = ActionClient(
        node,
        FollowJointTrajectory,
        args.action_name,
    )
    exit_code = send_goal_and_wait(
        node,
        client,
        goal,
        args.server_timeout,
        args.result_timeout,
    )
    if exit_code == 0:
        node.get_logger().info("Planned trajectory succeeded.")

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
