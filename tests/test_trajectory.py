"""Cancellation must be sent and acknowledged before reporting a stopped goal."""

from concurrent.futures import Future
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

try:
    from action_msgs.msg import GoalStatus
    from arm_control.trajectory import cancel_trajectory, execute_trajectory, trajectory_goal
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


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
