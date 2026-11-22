"""Planner and ROS adapter regressions; activate Conda and source the workspace."""

from concurrent.futures import Future
from contextlib import redirect_stdout
from dataclasses import asdict
import io
import math
import sys
from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock, patch

try:
    import rclpy  # noqa: F401 - check that the active Python can load ROS bindings
    from arm_msgs.srv import ExecutePlan
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash to test planning") from exc

from arm_control import trajectory
from arm_control.poses import load_named_poses
from arm_control.state import JointStateBuffer
from arm_description import JOINT_NAMES
from arm_planning import cli, service
from arm_planning.planner import JointSpacePlanner, PlanningOptions, WorkspaceBounds
from sensor_msgs.msg import JointState


class PlannerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = JointSpacePlanner.from_config()
        cls.poses = load_named_poses()

    def test_known_forward_kinematics(self):
        for values, expected in (
            ([0.0] * 6, (-0.8172, -0.3109, 0.0628)),
            (self.poses["home"], (0.0, -0.3109, 1.0794)),
        ):
            tcp = self.planner.model.tcp_position(values)
            for actual, reference in zip((tcp.x, tcp.y, tcp.z), expected):
                self.assertAlmostEqual(actual, reference, places=8)

    def test_plan_preserves_endpoint_step_and_velocity_limits(self):
        start = self.poses["home"]
        target = self.poses["inspection"]
        options = PlanningOptions()
        plan = self.planner.plan(start, [("inspection", target)], options)
        self.assertEqual(plan.waypoints[-1].positions, target)
        self.assertGreaterEqual(plan.total_duration, options.segment_duration)
        previous, previous_time = start, 0.0
        for point in plan.waypoints:
            dt = point.time_from_start - previous_time
            self.assertGreater(dt, 0)
            for name, old, new in zip(JOINT_NAMES, previous, point.positions):
                self.assertLessEqual(abs(new - old), options.max_joint_step + 1e-12)
                self.assertLessEqual(
                    abs(new - old) / dt,
                    self.planner.model.limits[name].velocity * options.velocity_scale + 1e-12,
                )
            previous, previous_time = point.positions, point.time_from_start

    def test_repeated_pose_has_no_motion(self):
        home = self.poses["home"]
        plan = self.planner.plan(home, [("home", home)] * 2, PlanningOptions())
        self.assertEqual(plan.waypoints, [])
        self.assertEqual(plan.total_duration, 0)
        self.assertEqual(len(plan.segments), 2)

    def test_invalid_inputs_are_rejected(self):
        for values in ([0.0] * 5, [math.nan] * 6, [math.inf] * 6, [100.0] * 6):
            with self.subTest(values=values), self.assertRaises(ValueError):
                self.planner.plan(self.poses["home"], [("invalid", values)], PlanningOptions())

    def test_workspace_limits_still_apply(self):
        planner = JointSpacePlanner(
            self.planner.model, WorkspaceBounds("base_footprint", 0.1, -0.2, 1.1),
        )
        with self.assertRaisesRegex(ValueError, "tcp radius"):
            planner.plan(self.poses["home"], [], PlanningOptions())

    def test_invalid_options_fail_before_interpolation(self):
        for options in (
            dict(max_joint_step=0), dict(segment_duration=-1),
            dict(velocity_scale=0), dict(velocity_scale=1.1),
            dict(joint_limit_margin=-0.1), dict(max_joint_step=math.nan),
        ):
            with self.subTest(options=options), self.assertRaises(ValueError):
                PlanningOptions(**options)


class PlanningServiceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = JointSpacePlanner.from_config()
        cls.poses = load_named_poses()

    def make_service_node(self):
        values = dict(asdict(PlanningOptions()), state_timeout=1, server_timeout=1, result_timeout=15)
        states = Mock()
        states.wait_for_complete_state.return_value = list(self.poses["home"])
        return SimpleNamespace(
            _poses=self.poses, _planner=self.planner, _option_defaults=asdict(PlanningOptions()),
            _execution_lock=threading.Lock(), _states=states, _action_client=Mock(),
            get_parameter=lambda name: SimpleNamespace(value=values[name]),
        )

    def test_repeated_pose_succeeds_without_sending_empty_goal(self):
        node = self.make_service_node()
        response = service.PlanningServiceNode._handle_execute_plan(
            node, ExecutePlan.Request(pose_names=["home", "home"]), ExecutePlan.Response(),
        )
        self.assertTrue(response.success)
        self.assertEqual(response.waypoint_count, 0)
        self.assertIn("no motion needed", response.message)
        node._action_client.wait_for_server.assert_not_called()
        self.assertFalse(node._execution_lock.locked())

    def test_movement_still_sends_goal(self):
        node = self.make_service_node()
        with patch.object(service, "execute_trajectory", return_value=(True, "Executed")) as execute:
            response = service.PlanningServiceNode._handle_execute_plan(
                node, ExecutePlan.Request(pose_names=["inspection"]), ExecutePlan.Response(),
            )
        self.assertTrue(response.success)
        self.assertGreater(response.waypoint_count, 0)
        execute.assert_called_once()

    def test_plan_only_does_not_execute(self):
        node = self.make_service_node()
        response = service.PlanningServiceNode._handle_execute_plan(
            node, ExecutePlan.Request(pose_names=["inspection"], plan_only=True),
            ExecutePlan.Response(),
        )
        self.assertTrue(response.success)
        node._action_client.wait_for_server.assert_not_called()

    def test_busy_service_rejects_concurrent_execution(self):
        node = self.make_service_node()
        node._execution_lock.acquire()
        response = service.PlanningServiceNode._handle_execute_plan(
            node, ExecutePlan.Request(pose_names=["inspection"]), ExecutePlan.Response(),
        )
        self.assertFalse(response.success)
        self.assertIn("busy", response.message)
        self.assertTrue(node._execution_lock.locked())

    def test_cli_repeated_pose_exits_successfully(self):
        with patch.object(sys, "argv", ["planner", "--pose", "home"]), \
                patch.object(cli, "rclpy") as ros, \
                patch.object(cli, "JointStateBuffer") as buffer, \
                patch.object(cli, "ActionClient") as action:
            buffer.return_value.wait_for_complete_state.return_value = list(self.poses["home"])
            self.assertEqual(cli.main(), 0)
            action.assert_not_called()
            ros.create_node.return_value.destroy_node.assert_called_once()
            ros.try_shutdown.assert_called_once()

    def test_cli_cleans_up_after_planning_error(self):
        with patch.object(sys, "argv", ["planner", "--pose", "home"]), \
                patch.object(cli, "rclpy") as ros, \
                patch.object(cli, "run", side_effect=ValueError("Invalid target")):
            self.assertEqual(cli.main(), 1)
            ros.create_node.return_value.destroy_node.assert_called_once()
            ros.try_shutdown.assert_called_once()

    def test_list_poses_does_not_load_robot_model(self):
        with patch.object(sys, "argv", ["planner", "--list-poses"]), \
                patch.object(cli.JointSpacePlanner, "from_config") as load_model, \
                redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(), 0)
            load_model.assert_not_called()

    def test_service_shutdown_is_safe_after_ros_signal(self):
        for interruption in (KeyboardInterrupt, service.ExternalShutdownException):
            with self.subTest(interruption=interruption), \
                    patch.object(service.rclpy, "init"), \
                    patch.object(service.rclpy, "try_shutdown") as shutdown, \
                    patch.object(service, "PlanningServiceNode") as node, \
                    patch.object(service, "MultiThreadedExecutor") as executor:
                executor.return_value.spin.side_effect = interruption()
                service.main()
                executor.return_value.shutdown.assert_called_once()
                node.return_value.destroy_node.assert_called_once()
                shutdown.assert_called_once()


class ControllerAdapterTest(unittest.TestCase):
    def test_joint_state_order_and_completeness(self):
        buffer = JointStateBuffer(Mock())
        buffer._receive(JointState(name=["joint_1"], position=[0.1]))
        self.assertIsNone(buffer.wait_for_complete_state(0))
        buffer._receive(JointState(name=list(reversed(JOINT_NAMES)), position=[6., 5., 4., 3., 2., 1.]))
        self.assertEqual(buffer.wait_for_complete_state(0), [1., 2., 3., 4., 5., 6.])

    def test_duration_rounding_carries_nanoseconds(self):
        goal = trajectory.trajectory_goal([([0.0] * 6, 1.9999999999)])
        self.assertEqual(goal.trajectory.points[0].time_from_start.sec, 2)
        self.assertEqual(goal.trajectory.points[0].time_from_start.nanosec, 0)

    def test_completed_future_does_not_wait_for_executor(self):
        future = Future()
        future.set_result("done")
        self.assertEqual(trajectory.wait_for_future(future, 0), "done")

    def test_pending_future_times_out(self):
        with self.assertRaises(TimeoutError):
            trajectory.wait_for_future(Future(), 0)

    def test_rejected_action_is_reported(self):
        future = Future()
        future.set_result(SimpleNamespace(accepted=False))
        client = Mock()
        client.send_goal_async.return_value = future
        success, message = trajectory.execute_trajectory(
            client, trajectory.trajectory_goal([([0.0] * 6, 1.0)]), 1, 1,
        )
        self.assertFalse(success)
        self.assertIn("rejected", message)


if __name__ == "__main__":
    unittest.main()
