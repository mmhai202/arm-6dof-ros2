"""Call the planning API without coupling the client to planner implementation."""

import argparse
import sys

from arm_control.trajectory import wait_for_future
from arm_msgs.srv import ExecutePlan, ListNamedPoses
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.utilities import remove_ros_args


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    targets = parser.add_mutually_exclusive_group(required=True)
    targets.add_argument("--list-poses", action="store_true")
    targets.add_argument("--pose", help="Target named pose.")
    targets.add_argument("--pose-sequence", nargs="+", help="Ordered named poses.")
    parser.add_argument("--plan-only", action="store_true")
    parser.add_argument("--service-timeout", type=float, default=10.0)
    return parser.parse_args(remove_ros_args()[1:])


def main():
    args = parse_args()
    if args.list_poses:
        service_type, name, request = ListNamedPoses, "list_named_poses", ListNamedPoses.Request()
        response_timeout = args.service_timeout
    else:
        service_type, name = ExecutePlan, "execute_plan"
        request = ExecutePlan.Request(
            pose_names=[args.pose] if args.pose is not None else args.pose_sequence,
            plan_only=args.plan_only,
        )
        response_timeout = args.service_timeout + 60.0

    rclpy.init()
    node = rclpy.create_node("arm_planning_service_client")
    try:
        client = node.create_client(service_type, f"/arm_planning/{name}")
        if not client.wait_for_service(timeout_sec=args.service_timeout):
            raise TimeoutError(f"{name} service is not ready.")
        response = wait_for_future(client.call_async(request), response_timeout, node=node)
        if args.list_poses:
            print("\n".join(response.pose_names))
            return 0
        print(response.message)
        return 0 if response.success else 1
    except (TimeoutError, RuntimeError) as error:
        print(f"[error] {error}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
