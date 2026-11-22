"""Plan and optionally execute a joint-space trajectory from the command line."""

import argparse
import sys

from arm_control.poses import add_target_arguments, load_named_poses, resolve_targets
from arm_control.state import JointStateBuffer
from arm_control.trajectory import ACTION_NAME, execute_trajectory, trajectory_goal
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args

from .planner import JointSpacePlanner, PlanningOptions


def parse_args(poses):
    defaults = PlanningOptions()
    parser = argparse.ArgumentParser(description=__doc__)
    add_target_arguments(parser, poses)
    parser.add_argument(
        "--segment-duration", type=float, default=defaults.segment_duration,
        help="Minimum seconds per segment.",
    )
    parser.add_argument(
        "--max-joint-step", type=float, default=defaults.max_joint_step,
        help="Maximum joint delta per waypoint (rad).",
    )
    parser.add_argument(
        "--velocity-scale", type=float, default=defaults.velocity_scale,
        help="Joint velocity scaling in (0, 1].",
    )
    parser.add_argument(
        "--joint-limit-margin", type=float, default=defaults.joint_limit_margin,
        help="Margin inside joint limits (rad).",
    )
    parser.add_argument("--state-timeout", type=float, default=10.0)
    parser.add_argument("--server-timeout", type=float, default=20.0)
    parser.add_argument("--result-timeout", type=float, default=30.0)
    parser.add_argument("--plan-only", action="store_true", help="Plan without executing.")
    parser.add_argument("--action-name", default=ACTION_NAME)
    return parser.parse_args(remove_ros_args()[1:])


def run(node, args, targets, options):
    current = JointStateBuffer(node).wait_for_complete_state(args.state_timeout, spin=True)
    if current is None:
        raise ValueError("Timed out while waiting for /joint_states.")
    plan = JointSpacePlanner.from_config().plan(current, targets, options)
    node.get_logger().info(plan.summary)
    for segment in plan.segments:
        tcp = segment.tcp
        node.get_logger().info(
            f"Segment '{segment.name}': {segment.waypoint_count} point(s), "
            f"max joint delta {segment.max_delta:.3f} rad, duration {segment.duration:.2f}s, "
            f"tcp (x={tcp.x:.3f}, y={tcp.y:.3f}, z={tcp.z:.3f}, r={tcp.radius:.3f}) m."
        )
    if not plan.waypoints:
        node.get_logger().info("Already at the requested pose; no motion needed.")
        return 0
    if args.plan_only:
        node.get_logger().info("Plan-only mode enabled; skipping execution.")
        return 0
    client = ActionClient(node, FollowJointTrajectory, args.action_name)
    goal = trajectory_goal((point.positions, point.time_from_start) for point in plan.waypoints)
    success, message = execute_trajectory(
        client, goal, args.server_timeout, args.result_timeout, node=node,
    )
    log = node.get_logger().info if success else node.get_logger().error
    log(message)
    return 0 if success else 1


def main():
    poses = load_named_poses()
    args = parse_args(poses)
    if args.list_poses:
        print("\n".join(sorted(poses)))
        return 0
    try:
        targets = resolve_targets(args, poses)
        options = PlanningOptions(
            segment_duration=args.segment_duration, max_joint_step=args.max_joint_step,
            velocity_scale=args.velocity_scale, joint_limit_margin=args.joint_limit_margin,
        )
    except ValueError as error:
        print(f"[error] {error}", file=sys.stderr)
        return 1

    rclpy.init()
    node = rclpy.create_node("arm_joint_space_planner")
    try:
        return run(node, args, targets, options)
    except (ValueError, OSError, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
