"""ROS planning API; computation lives in planner, controller I/O in arm_control."""

from dataclasses import asdict
import threading

from arm_control.poses import load_named_poses
from arm_control.state import JointStateBuffer
from arm_control.trajectory import ACTION_NAME, execute_trajectory, trajectory_goal
from arm_msgs.srv import ExecutePlan, ListNamedPoses
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from .planner import JointSpacePlanner, PlanningOptions


class PlanningServiceNode(Node):
    def __init__(self):
        super().__init__("arm_planning_service")
        self._poses = load_named_poses()
        self._planner = JointSpacePlanner.from_config()
        self._option_defaults = asdict(PlanningOptions())
        self.declare_parameters("", [
            ("action_name", ACTION_NAME), ("state_timeout", 10.0),
            ("server_timeout", 10.0), ("result_timeout", 45.0),
            *self._option_defaults.items(),
        ])
        self._execution_lock = threading.Lock()
        callbacks = ReentrantCallbackGroup()
        self._states = JointStateBuffer(self, callback_group=callbacks)
        self._action_client = ActionClient(
            self, FollowJointTrajectory, self.get_parameter("action_name").value,
            callback_group=callbacks,
        )
        self.create_service(
            ListNamedPoses, "/arm_planning/list_named_poses", self._handle_list_named_poses,
            callback_group=callbacks,
        )
        self.create_service(
            ExecutePlan, "/arm_planning/execute_plan", self._handle_execute_plan,
            callback_group=callbacks,
        )

    def _handle_list_named_poses(self, request, response):
        response.pose_names = sorted(self._poses)
        return response

    def _handle_execute_plan(self, request, response):
        if not request.pose_names:
            response.message = "pose_names must not be empty."
            return response
        unknown = [name for name in request.pose_names if name not in self._poses]
        if unknown:
            response.message = f"Unknown pose(s): {', '.join(unknown)}"
            return response
        if not self._execution_lock.acquire(blocking=False):
            response.message = "Planner is busy executing another request."
            return response

        try:
            current = self._states.wait_for_complete_state(
                self.get_parameter("state_timeout").value,
            )
            if current is None:
                raise ValueError("Timed out while waiting for /joint_states.")
            options = PlanningOptions(**{
                name: self.get_parameter(name).value for name in self._option_defaults
            })
            targets = [(name, self._poses[name]) for name in request.pose_names]
            plan = self._planner.plan(current, targets, options)
            response.waypoint_count = len(plan.waypoints)
            response.total_duration = plan.total_duration
            if request.plan_only:
                response.success = True
                response.message = f"{plan.summary} Execution skipped because plan_only=true."
            else:
                goal = trajectory_goal(
                    (point.positions, point.time_from_start) for point in plan.waypoints
                )
                response.success, message = execute_trajectory(
                    self._action_client, goal, self.get_parameter("server_timeout").value,
                    self.get_parameter("result_timeout").value,
                )
                response.message = f"{plan.summary} {message}"
        except (ValueError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
        finally:
            self._execution_lock.release()
        return response


def main():
    rclpy.init()
    node = PlanningServiceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
