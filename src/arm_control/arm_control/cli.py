"""Send named poses or joint positions directly to arm_controller."""

import argparse
import sys

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from .poses import add_target_arguments, load_named_poses, resolve_targets
from .trajectory import ACTION_NAME, execute_trajectory, trajectory_goal


def parse_args(poses):
    parser = argparse.ArgumentParser(description=__doc__)
    add_target_arguments(parser, poses)
    parser.add_argument("--duration", type=int, default=3, help="Seconds to reach each target.")
    parser.add_argument("--server-timeout", type=float, default=20.0)
    parser.add_argument("--result-timeout", type=float, default=15.0)
    parser.add_argument(
        "--wait-only", action="store_true", help="Only check action server readiness.",
    )
    return parser.parse_args(remove_ros_args()[1:])


def main():
    poses = load_named_poses()
    args = parse_args(poses)
    if args.list_poses:
        print("\n".join(sorted(poses)))
        return 0
    try:
        if args.duration <= 0:
            raise ValueError("--duration must be positive.")
        targets = [] if args.wait_only else resolve_targets(args, poses)
    except ValueError as error:
        print(f"[error] {error}", file=sys.stderr)
        return 1

    # Keep ROS alive while Ctrl+C cancels an accepted controller goal.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("arm_trajectory_demo_client")
    try:
        client = ActionClient(node, FollowJointTrajectory, ACTION_NAME)
        if args.wait_only:
            if not client.wait_for_server(timeout_sec=args.server_timeout):
                raise TimeoutError("Action server is not ready.")
            node.get_logger().info("Action server is ready.")
            return 0
        for name, positions in targets:
            node.get_logger().info(f"Sending pose '{name}'.")
            goal = trajectory_goal([(positions, args.duration)])
            success, message = execute_trajectory(
                client, goal, args.server_timeout, args.result_timeout, node=node,
            )
            if not success:
                node.get_logger().error(message)
                return 1
            node.get_logger().info(f"Pose '{name}' succeeded.")
        return 0
    except (TimeoutError, RuntimeError) as error:
        node.get_logger().error(str(error))
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
