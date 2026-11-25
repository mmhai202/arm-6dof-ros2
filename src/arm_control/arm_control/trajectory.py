"""ROS trajectory messages and action handling shared by CLI and service nodes."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from arm_description import JOINT_NAMES
from arm_description.resources import load_yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from trajectory_msgs.msg import JointTrajectoryPoint


ACTION_NAME = "/arm_controller/follow_joint_trajectory"


class MotionStopError(RuntimeError):
    """The controller did not confirm that an interrupted motion ended."""


def controller_start_tolerance():
    """MoveIt's scalar start bound must accept the controllers' completed goals.

    Controllers still enforce their individual joint position and stopped
    velocity tolerances. This bound avoids a stricter second completion check
    when the task submits the next, separately planned trajectory.
    """
    config = load_yaml("arm_control", "config/controllers.yaml")
    tolerances = []
    for name in ("arm_controller", "gripper_controller"):
        parameters = config[name]["ros__parameters"]
        for joint in parameters["joints"]:
            value = float(parameters["constraints"][joint]["goal"])
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name}.{joint}.goal must be finite and positive.")
            tolerances.append(value)
    return max(tolerances)


def trajectory_goal(waypoints, *, joint_names=JOINT_NAMES):
    """Convert (positions, seconds_from_start) pairs into a controller goal."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joint_names)
    for positions, seconds in waypoints:
        sec, nanosec = divmod(round(seconds * 1_000_000_000), 1_000_000_000)
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=list(positions), time_from_start=Duration(sec=sec, nanosec=nanosec),
        ))
    return goal


def wait_for_future(future, timeout, *, node=None, cancel_check=None):
    """Spin a CLI node, or wait for a service node's existing executor."""
    if future.done():
        return future.result()
    if node is not None and cancel_check is not None:
        deadline = time.monotonic() + timeout
        while not future.done() and node.context.ok():
            cancel_check()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
    elif node is not None:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    else:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout=timeout)
    if not future.done():
        raise TimeoutError("ROS request timed out.")
    return future.result()


def cancel_trajectory(handle, *, node=None, timeout=3.0):
    """Request cancellation and confirm a terminal result within bounded waits."""
    try:
        result = handle.get_result_async()
        if not result.done():
            wait_for_future(handle.cancel_goal_async(), timeout, node=node)
        response = wait_for_future(result, timeout, node=node)
        return response.status in (
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED,
        )
    except (TimeoutError, RuntimeError):
        return False


def execute_trajectory(client, goal, server_timeout, result_timeout, *, node=None):
    """Return (success, message); callbacks use the already-running executor."""
    if not goal.trajectory.points:
        return True, "Already at the requested pose; no motion needed."
    if not client.wait_for_server(timeout_sec=server_timeout):
        return False, "Action server is not ready."
    try:
        handle = wait_for_future(client.send_goal_async(goal), server_timeout, node=node)
    except TimeoutError:
        return False, "Timed out while sending the planned trajectory."
    if handle is None or not handle.accepted:
        return False, "Planned trajectory was rejected."
    try:
        result = wait_for_future(handle.get_result_async(), result_timeout, node=node).result
    except TimeoutError:
        stopped = cancel_trajectory(handle, node=node)
        detail = "Cancellation confirmed." if stopped else "Could not confirm cancellation."
        return False, f"Timed out while waiting for trajectory execution. {detail}"
    except KeyboardInterrupt:
        if not cancel_trajectory(handle, node=node):
            raise MotionStopError("Interrupted, but could not confirm trajectory cancellation.")
        raise
    if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        return False, (
            f"Trajectory failed. error_code={result.error_code}, "
            f"error_string='{result.error_string}'"
        )
    return True, "Trajectory execution succeeded."
