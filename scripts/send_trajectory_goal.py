#!/usr/bin/env python3

import argparse
import sys

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]

POSE_PRESETS = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "reach_forward": [0.0, -0.5, 0.8, 0.0, 0.4, 0.0],
    "reach_high": [0.1, -1.0, 1.2, 0.0, 0.6, 0.0],
    "left_reach": [0.8, -0.7, 1.0, 0.2, 0.5, 0.1],
    "wrist_down": [0.2, -0.9, 1.1, 0.0, -0.6, 0.0],
    "inspection": [0.6, -0.8, 1.0, 0.0, 0.8, 0.2],
}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Send a trajectory goal to the baseline arm_controller."
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
        "--list-poses",
        action="store_true",
        help="Print the available pose presets and exit.",
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=3,
        help="Seconds to reach the single trajectory point.",
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
        default=15.0,
        help="Seconds to wait for the action result.",
    )
    parser.add_argument(
        "--wait-only",
        action="store_true",
        help="Only check whether the action server is ready without sending a goal.",
    )
    return parser.parse_args()


def build_goal(positions, duration_seconds):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES

    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=duration_seconds)

    goal.trajectory.points = [point]
    return goal


def main():
    args = parse_args()

    if args.list_poses:
        for pose_name in sorted(POSE_PRESETS.keys()):
            print(pose_name)
        return 0

    rclpy.init()
    node = rclpy.create_node("arm_trajectory_demo_client")
    client = ActionClient(
        node,
        FollowJointTrajectory,
        "/arm_controller/follow_joint_trajectory",
    )

    if not client.wait_for_server(timeout_sec=args.server_timeout):
        node.get_logger().error("Action server is not ready.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if args.wait_only:
        node.get_logger().info("Action server is ready.")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    positions = args.positions
    if args.pose is not None:
        positions = POSE_PRESETS[args.pose]

    if positions is None:
        node.get_logger().error(
            "--positions or --pose is required unless --wait-only is used."
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    goal = build_goal(positions, args.duration)
    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(
        node,
        send_goal_future,
        timeout_sec=args.server_timeout,
    )

    if not send_goal_future.done():
        node.get_logger().error("Timed out while sending the goal.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    goal_handle = send_goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("Goal was rejected.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(
        node,
        result_future,
        timeout_sec=args.result_timeout,
    )

    if not result_future.done():
        node.get_logger().error("Timed out while waiting for the result.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    result = result_future.result()
    error_code = result.result.error_code
    error_string = result.result.error_string

    if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        node.get_logger().error(
            f"Goal failed. error_code={error_code}, error_string='{error_string}'"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.get_logger().info("Goal succeeded.")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
