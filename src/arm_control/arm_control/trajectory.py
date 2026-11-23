"""ROS trajectory messages and action handling shared by CLI and service nodes."""

import threading

from arm_description import JOINT_NAMES
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import rclpy
from trajectory_msgs.msg import JointTrajectoryPoint


ACTION_NAME = "/arm_controller/follow_joint_trajectory"


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


def wait_for_future(future, timeout, *, node=None):
    """Spin a CLI node, or wait for a service node's existing executor."""
    if future.done():
        return future.result()
    if node is not None:
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    else:
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        completed.wait(timeout=timeout)
    if not future.done():
        raise TimeoutError("ROS request timed out.")
    return future.result()


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
        return False, "Timed out while waiting for trajectory execution."
    if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
        return False, (
            f"Trajectory failed. error_code={result.error_code}, "
            f"error_string='{result.error_string}'"
        )
    return True, "Trajectory execution succeeded."
