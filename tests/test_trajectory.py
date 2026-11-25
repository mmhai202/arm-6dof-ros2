"""Cancellation must be sent and acknowledged before reporting a stopped goal."""

from concurrent.futures import Future
import copy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    from action_msgs.msg import GoalStatus
    from arm_control.trajectory import (
        cancel_trajectory, controller_start_tolerance, execute_trajectory, trajectory_goal,
    )
    from arm_description.resources import load_yaml
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


class ControllerToleranceTest(unittest.TestCase):
    def test_start_validation_accepts_every_configured_controller_goal(self):
        config = load_yaml("arm_control", "config/controllers.yaml")
        tolerance = controller_start_tolerance()
        self.assertAlmostEqual(tolerance, 0.02)
        for name in ("arm_controller", "gripper_controller"):
            parameters = config[name]["ros__parameters"]
            for joint in parameters["joints"]:
                self.assertLessEqual(parameters["constraints"][joint]["goal"], tolerance)
        # A controller configuration change must also change MoveIt's bound.
        changed = copy.deepcopy(config)
        changed["arm_controller"]["ros__parameters"]["constraints"]["joint_2"]["goal"] = 0.03
        with patch("arm_control.trajectory.load_yaml", return_value=changed):
            self.assertAlmostEqual(controller_start_tolerance(), 0.03)

    def test_unbounded_or_invalid_goal_tolerances_are_rejected(self):
        config = load_yaml("arm_control", "config/controllers.yaml")
        for value in (0.0, -1.0, float("nan"), float("inf")):
            with self.subTest(value=value):
                config["arm_controller"]["ros__parameters"]["constraints"]["joint_1"]["goal"] = value
                with patch("arm_control.trajectory.load_yaml", return_value=config):
                    with self.assertRaisesRegex(ValueError, "finite and positive"):
                        controller_start_tolerance()


class TrajectoryCancellationTest(unittest.TestCase):
    def test_execution_timeout_requests_and_confirms_cancellation(self):
        client = Mock()
        handle = Mock(accepted=True, get_result_async=Mock(return_value=Future()))
        with patch("arm_control.trajectory.wait_for_future", side_effect=[
            handle, TimeoutError(), SimpleNamespace(),
            SimpleNamespace(status=GoalStatus.STATUS_CANCELED),
        ]):
            success, message = execute_trajectory(client, trajectory_goal([([0.0]*6, 10)]), 1, 1)
        self.assertFalse(success)
        self.assertIn("Cancellation confirmed", message)
        handle.cancel_goal_async.assert_called_once()

    def test_interrupt_cancels_goal_before_propagating(self):
        handle = Mock(accepted=True, get_result_async=Mock(return_value=Future()))
        with patch("arm_control.trajectory.wait_for_future", side_effect=[
            handle, KeyboardInterrupt(), SimpleNamespace(),
            SimpleNamespace(status=GoalStatus.STATUS_CANCELED),
        ]):
            with self.assertRaises(KeyboardInterrupt):
                execute_trajectory(Mock(), trajectory_goal([([0.0]*6, 10)]), 1, 1)
        handle.cancel_goal_async.assert_called_once()

    def test_unconfirmed_cancellation_is_reported(self):
        handle = Mock(accepted=True, get_result_async=Mock(return_value=Future()))
        with patch("arm_control.trajectory.wait_for_future", side_effect=[
            handle, TimeoutError(), TimeoutError(),
        ]):
            success, message = execute_trajectory(Mock(), trajectory_goal([([0.0]*6, 10)]), 1, 1)
        self.assertFalse(success)
        self.assertIn("Could not confirm cancellation", message)

    def test_interrupt_does_not_report_success_when_cancellation_is_unconfirmed(self):
        handle = Mock(accepted=True, get_result_async=Mock(return_value=Future()))
        with patch("arm_control.trajectory.wait_for_future", side_effect=[
            handle, KeyboardInterrupt(), TimeoutError(),
        ]):
            with self.assertRaisesRegex(RuntimeError, "could not confirm"):
                execute_trajectory(Mock(), trajectory_goal([([0.0]*6, 10)]), 1, 1)

    def test_acknowledging_cancel_without_terminal_result_is_not_enough(self):
        handle = Mock(get_result_async=Mock(return_value=Future()))
        with patch("arm_control.trajectory.wait_for_future", side_effect=[
            SimpleNamespace(), TimeoutError(),
        ]):
            self.assertFalse(cancel_trajectory(handle))

    def test_completed_goal_needs_no_cancellation(self):
        future = Future()
        future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))
        handle = Mock(get_result_async=Mock(return_value=future))
        self.assertTrue(cancel_trajectory(handle))
        handle.cancel_goal_async.assert_not_called()
