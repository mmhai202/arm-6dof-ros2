#!/usr/bin/env python3

import argparse
import sys

from arm_msgs.srv import ExecutePlan, ListNamedPoses
import rclpy
from rclpy.node import Node


def parse_args():
    parser = argparse.ArgumentParser(
        description="Call the project-owned planning services."
    )
    parser.add_argument(
        "--list-poses",
        action="store_true",
        help="List the named poses exposed by the planning service.",
    )
    parser.add_argument(
        "--pose",
        help="Execute a single named pose through the planning service.",
    )
    parser.add_argument(
        "--pose-sequence",
        nargs="+",
        help="Execute a named pose sequence through the planning service.",
    )
    parser.add_argument(
        "--plan-only",
        action="store_true",
        help="Ask the service to plan only without execution.",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for each service to appear and respond.",
    )
    return parser.parse_args()


class PlanningServiceClient(Node):
    def __init__(self):
        super().__init__("arm_planning_service_client")
        self._list_client = self.create_client(
            ListNamedPoses, "/arm_planning/list_named_poses"
        )
        self._execute_client = self.create_client(
            ExecutePlan, "/arm_planning/execute_plan"
        )


def wait_for_service(client, timeout_sec):
    return client.wait_for_service(timeout_sec=timeout_sec)


def main():
    args = parse_args()
    if not args.list_poses and args.pose is None and args.pose_sequence is None:
        print(
            "[error] Use --list-poses, --pose, or --pose-sequence.",
            file=sys.stderr,
        )
        return 1

    if args.pose is not None and args.pose_sequence is not None:
        print("[error] --pose cannot be combined with --pose-sequence.", file=sys.stderr)
        return 1

    rclpy.init()
    node = PlanningServiceClient()

    try:
        if args.list_poses:
            if not wait_for_service(node._list_client, args.service_timeout):
                print("[error] list_named_poses service is not ready.", file=sys.stderr)
                return 1

            request = ListNamedPoses.Request()
            future = node._list_client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.service_timeout)
            if not future.done():
                print("[error] Timed out while calling list_named_poses.", file=sys.stderr)
                return 1

            response = future.result()
            for pose_name in response.pose_names:
                print(pose_name)
            return 0

        if not wait_for_service(node._execute_client, args.service_timeout):
            print("[error] execute_plan service is not ready.", file=sys.stderr)
            return 1

        request = ExecutePlan.Request()
        if args.pose is not None:
            request.pose_names = [args.pose]
        else:
            request.pose_names = args.pose_sequence
        request.plan_only = args.plan_only

        future = node._execute_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.service_timeout + 60.0)
        if not future.done():
            print("[error] Timed out while calling execute_plan.", file=sys.stderr)
            return 1

        response = future.result()
        print(response.message)
        return 0 if response.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
