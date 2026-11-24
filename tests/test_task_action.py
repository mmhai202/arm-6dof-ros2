"""Admission, progress and terminal results without a running simulation."""

from concurrent.futures import Future, ThreadPoolExecutor
import math
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from arm_control.trajectory import MotionStopError, wait_for_future
from arm_moveit_config.task_server import PickPlaceServer, goal_motion
from arm_moveit_config.task_state import TaskCanceled, TaskProgress
from arm_task_interfaces.action import PickPlace
from rclpy.action import GoalResponse


class TaskActionTest(unittest.TestCase):
    def server(self):
        return SimpleNamespace(gate=threading.Lock(), busy=False, blocked=False,
                               stopping=threading.Event(), perception_options={},
                               get_logger=Mock())

    def handle(self):
        return Mock(request=PickPlace.Goal(), is_cancel_requested=False)

    def worker(self):
        return Mock(stage="return_home", holding=False, fault=None)

    def test_default_goal_and_motion_validation(self):
        self.assertEqual(goal_motion(PickPlace.Goal()), {})
        goal = PickPlace.Goal(use_default_motion=False, velocity_scale=0.3)
        self.assertEqual(goal_motion(goal)["velocity_scale"], 0.3)
        for value in (0.0, -0.1, 1.1, math.nan, math.inf):
            goal.velocity_scale = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                goal_motion(goal)

    def test_concurrent_clients_cannot_both_reserve_a_task(self):
        server = self.server()
        barrier = threading.Barrier(4)

        def submit(_):
            barrier.wait(timeout=5)
            return PickPlaceServer.accept_goal(server, PickPlace.Goal())

        with ThreadPoolExecutor(max_workers=4) as pool:
            replies = list(pool.map(submit, range(4)))
        self.assertEqual(replies.count(GoalResponse.ACCEPT), 1)
        self.assertEqual(replies.count(GoalResponse.REJECT), 3)

    def test_invalid_goal_never_reserves_the_worker(self):
        server = self.server()
        reply = PickPlaceServer.accept_goal(server, PickPlace.Goal(
            use_default_motion=False, velocity_scale=0.0))
        self.assertEqual(reply, GoalResponse.REJECT)
        self.assertFalse(server.busy)

    def test_success_publishes_typed_progress_and_drains_before_result(self):
        server, handle, worker = self.server(), self.handle(), self.worker()
        server.busy = True

        def run(mode):
            worker.progress_callback(TaskProgress("executing", "lift", 8, 17))

        def drain():
            handle.succeed.assert_not_called()
            self.assertTrue(server.busy)

        worker.run.side_effect = run
        worker.stop_sync.side_effect = drain
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=worker):
            result = PickPlaceServer.execute(server, handle)
        self.assertTrue(result.success)
        self.assertEqual(result.final_stage, "return_home")
        feedback = handle.publish_feedback.call_args.args[0]
        self.assertEqual((feedback.phase, feedback.stage, feedback.completed_stages),
                         ("executing", "lift", 8))
        handle.succeed.assert_called_once()
        worker.ownership.close.assert_called_once()
        self.assertFalse(server.busy)

    def test_camera_mode_reaches_worker_without_global_ros_arguments(self):
        server, handle = self.server(), self.handle()
        server.perception_options = {"enabled": True, "randomize_spawn": True, "spawn_seed": 17}
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=self.worker()) as factory:
            self.assertTrue(PickPlaceServer.execute(server, handle).success)
        options = factory.call_args.kwargs
        self.assertEqual(options["perception_overrides"], server.perception_options)
        self.assertFalse(options["use_global_arguments"])

    def test_cancel_is_terminal_only_after_cleanup(self):
        server, handle, worker = self.server(), self.handle(), self.worker()
        server.busy = True
        handle.is_cancel_requested = True
        worker.run.side_effect = TaskCanceled("requested")
        worker.stop_sync.side_effect = lambda: handle.canceled.assert_not_called()
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=worker):
            result = PickPlaceServer.execute(server, handle)
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, PickPlace.Result.CANCELED)
        handle.canceled.assert_called_once()
        worker.retain_after_failure.assert_called_once()
        self.assertFalse(server.busy)

    def test_cleanup_failure_aborts_instead_of_claiming_cancellation(self):
        server, handle, worker = self.server(), self.handle(), self.worker()
        handle.is_cancel_requested = True
        worker.run.side_effect = TaskCanceled("requested")
        worker.stop_sync.side_effect = RuntimeError("unacknowledged scene update")
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=worker):
            result = PickPlaceServer.execute(server, handle)
        self.assertEqual(result.error_code, PickPlace.Result.FAILED)
        handle.abort.assert_called_once()
        handle.canceled.assert_not_called()
        self.assertIn("Cleanup failed", result.message)
        self.assertFalse(server.busy)

    def test_failed_scene_acquisition_does_not_write_to_another_owners_scene(self):
        server, handle, worker = self.server(), self.handle(), self.worker()
        worker.acquire_scene.side_effect = RuntimeError("another owner")
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=worker):
            result = PickPlaceServer.execute(server, handle)
        self.assertFalse(result.success)
        worker.run.assert_not_called()
        worker.retain_after_failure.assert_not_called()
        worker.stop_sync.assert_not_called()
        handle.abort.assert_called_once()

    def test_worker_construction_failure_releases_admission_gate(self):
        server = self.server()
        server.busy = True
        with patch("arm_moveit_config.task_server.GazeboShelfTask", side_effect=RuntimeError("setup")):
            result = PickPlaceServer.execute(server, self.handle())
        self.assertFalse(result.success)
        self.assertFalse(server.busy)

    def test_unconfirmed_stop_blocks_new_goals(self):
        server, worker = self.server(), self.worker()
        worker.run.side_effect = MotionStopError("stop not confirmed")
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=worker):
            result = PickPlaceServer.execute(server, self.handle())
        self.assertEqual(result.error_code, PickPlace.Result.FAILED)
        self.assertTrue(server.blocked)
        self.assertEqual(PickPlaceServer.accept_goal(server, PickPlace.Goal()), GoalResponse.REJECT)

    def test_worker_destruction_failure_still_closes_lock_and_releases_gate(self):
        server, worker = self.server(), self.worker()
        server.busy = True
        worker.destroy_node.side_effect = RuntimeError("destroy failed")
        with patch("arm_moveit_config.task_server.GazeboShelfTask", return_value=worker):
            result = PickPlaceServer.execute(server, self.handle())
        self.assertEqual(result.error_code, PickPlace.Result.FAILED)
        worker.ownership.close.assert_called_once()
        self.assertFalse(server.busy)
        self.assertTrue(server.blocked)

    def test_planning_wait_can_be_canceled_before_a_service_responds(self):
        node = SimpleNamespace(context=SimpleNamespace(ok=lambda: True))
        cancel = Mock(side_effect=TaskCanceled("requested"))
        with patch("arm_control.trajectory.rclpy.spin_once") as spin:
            with self.assertRaises(TaskCanceled):
                wait_for_future(Future(), 45, node=node, cancel_check=cancel)
        spin.assert_not_called()
