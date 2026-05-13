#!/usr/bin/env python3

import threading
import time

from arm_msgs.srv import ExecutePlan, ListNamedPoses
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from plan_joint_trajectory import (
    POSE_PRESETS,
    build_trajectory,
)


class PlanningServiceNode(Node):
    def __init__(self):
        super().__init__("arm_planning_service")

        self.declare_parameter("action_name", "/arm_controller/follow_joint_trajectory")
        self.declare_parameter("state_timeout", 10.0)
        self.declare_parameter("server_timeout", 10.0)
        self.declare_parameter("result_timeout", 45.0)
        self.declare_parameter("segment_duration", 2.5)
        self.declare_parameter("max_joint_step", 0.2)
        self.declare_parameter("velocity_scale", 0.6)
        self.declare_parameter("joint_limit_margin", 0.02)

        self._callback_group = ReentrantCallbackGroup()
        self._latest_positions = None
        self._joint_state_event = threading.Event()
        self._execution_lock = threading.Lock()

        self.create_subscription(
            JointState,
            "/joint_states",
            self._handle_joint_state,
            10,
            callback_group=self._callback_group,
        )

        action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            action_name,
            callback_group=self._callback_group,
        )

        self.create_service(
            ListNamedPoses,
            "/arm_planning/list_named_poses",
            self._handle_list_named_poses,
            callback_group=self._callback_group,
        )
        self.create_service(
            ExecutePlan,
            "/arm_planning/execute_plan",
            self._handle_execute_plan,
            callback_group=self._callback_group,
        )

    def _handle_joint_state(self, message):
        joint_positions = {
            joint_name: position
            for joint_name, position in zip(message.name, message.position)
            if joint_name.startswith("joint_")
        }

        expected_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        if all(joint_name in joint_positions for joint_name in expected_joint_names):
            self._latest_positions = [joint_positions[joint_name] for joint_name in expected_joint_names]
            self._joint_state_event.set()

    def _handle_list_named_poses(self, request, response):
        del request
        response.pose_names = sorted(POSE_PRESETS.keys())
        return response

    def _handle_execute_plan(self, request, response):
        if not request.pose_names:
            response.success = False
            response.message = "pose_names must not be empty."
            return response

        unknown_poses = [pose_name for pose_name in request.pose_names if pose_name not in POSE_PRESETS]
        if unknown_poses:
            response.success = False
            response.message = f"Unknown pose(s): {', '.join(unknown_poses)}"
            return response

        if not self._execution_lock.acquire(blocking=False):
            response.success = False
            response.message = "Planner is busy executing another request."
            return response

        try:
            current_positions = self._wait_for_joint_state(
                self.get_parameter("state_timeout").value
            )
            if current_positions is None:
                response.success = False
                response.message = "Timed out while waiting for /joint_states."
                return response

            target_sequence = [
                (pose_name, list(POSE_PRESETS[pose_name]))
                for pose_name in request.pose_names
            ]
            goal, segment_summaries, total_duration = build_trajectory(
                current_positions,
                target_sequence,
                self.get_parameter("segment_duration").value,
                self.get_parameter("max_joint_step").value,
                self.get_parameter("velocity_scale").value,
                self.get_parameter("joint_limit_margin").value,
            )

            response.waypoint_count = len(goal.trajectory.points)
            response.total_duration = total_duration
            summary = (
                f"Planned {len(goal.trajectory.points)} waypoint(s) across "
                f"{len(segment_summaries)} segment(s) in {total_duration:.2f}s."
            )

            if request.plan_only:
                response.success = True
                response.message = f"{summary} Execution skipped because plan_only=true."
                return response

            success, message = self._send_goal_and_wait(
                goal,
                self.get_parameter("server_timeout").value,
                self.get_parameter("result_timeout").value,
            )
            response.success = success
            response.message = f"{summary} {message}"
            return response
        except ValueError as error:
            response.success = False
            response.message = str(error)
            return response
        finally:
            self._execution_lock.release()

    def _wait_for_joint_state(self, timeout_sec):
        if self._latest_positions is not None:
            return list(self._latest_positions)

        if self._joint_state_event.wait(timeout=timeout_sec):
            return list(self._latest_positions)

        return None

    @staticmethod
    def _wait_for_future(future, timeout_sec):
        future_event = threading.Event()
        future.add_done_callback(lambda _: future_event.set())
        if future.done():
            return True
        return future_event.wait(timeout=timeout_sec)

    def _send_goal_and_wait(self, goal, server_timeout, result_timeout):
        if not self._action_client.wait_for_server(timeout_sec=server_timeout):
            return False, "Action server is not ready."

        send_goal_future = self._action_client.send_goal_async(goal)
        if not self._wait_for_future(send_goal_future, server_timeout):
            return False, "Timed out while sending the planned trajectory."

        goal_handle = send_goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return False, "Planned trajectory was rejected."

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future, result_timeout):
            return False, "Timed out while waiting for trajectory execution."

        result = result_future.result()
        error_code = result.result.error_code
        error_string = result.result.error_string

        if error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False, (
                f"Trajectory failed. error_code={error_code}, error_string='{error_string}'"
            )

        return True, "Trajectory execution succeeded."


def main():
    rclpy.init()
    node = PlanningServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
