"""ARM_RUN_TASK_ACTION_TESTS=1 with a disposable --pick-place session."""

import os
import time
import unittest


ENABLED = os.environ.get("ARM_RUN_TASK_ACTION_TESTS") == "1"
if ENABLED:
    from action_msgs.msg import GoalStatus
    from arm_description import JOINT_NAMES
    from arm_task_interfaces.action import PickPlace
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState


@unittest.skipUnless(ENABLED, "Set ARM_RUN_TASK_ACTION_TESTS=1 with --pick-place running.")
class TaskActionIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("task_action_regression")
        cls.client = ActionClient(cls.node, PickPlace, "/shelf_pick_place")
        cls.joints = None
        cls.subscription = cls.node.create_subscription(
            JointState, "/joint_states", lambda msg: setattr(cls, "joints", msg), qos_profile_sensor_data)
        if not cls.client.wait_for_server(timeout_sec=30):
            raise RuntimeError("Pick-and-place Action server unavailable")

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def setUp(self):
        self.feedback = []
        self.active = None

    def tearDown(self):
        if self.active is not None:
            self.wait(self.active.cancel_goal_async(), 10)
            self.wait(self.active.get_result_async(), 60)

    def wait(self, future, timeout=20):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), "Action operation timed out")
        return future.result()

    def wait_for(self, predicate, timeout=60):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertTrue(predicate(), "Expected task state was not observed")

    def send(self, goal=None):
        handle = self.wait(self.client.send_goal_async(
            goal or PickPlace.Goal(), feedback_callback=lambda msg: self.feedback.append(msg.feedback)))
        self.assertTrue(handle.accepted)
        self.active = handle
        return handle

    def cancel(self, handle):
        reply = self.wait(handle.cancel_goal_async())
        self.assertTrue(reply.goals_canceling)
        response = self.wait(handle.get_result_async(), 60)
        self.active = None
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED, response.result.message)
        self.assertFalse(response.result.success)
        self.assertEqual(response.result.error_code, PickPlace.Result.CANCELED)

    def test_cancel_during_execution_stops_motion(self):
        self.wait_for(lambda: self.joints is not None)
        before = dict(zip(self.joints.name, self.joints.position))
        handle = self.send(PickPlace.Goal(use_default_motion=False, velocity_scale=0.15))

        def moving():
            current = dict(zip(self.joints.name, self.joints.position))
            return (any(msg.phase == "executing" and msg.stage == "outside_shelf" for msg in self.feedback)
                    and max(abs(current[name] - before[name]) for name in JOINT_NAMES) > 0.04)

        self.wait_for(moving)
        self.cancel(handle)
        deadline = time.monotonic() + 0.5
        self.wait_for(lambda: time.monotonic() >= deadline)
        stopped = dict(zip(self.joints.name, self.joints.position))
        deadline = time.monotonic() + 1.0
        self.wait_for(lambda: time.monotonic() >= deadline)
        current = dict(zip(self.joints.name, self.joints.position))
        for name in JOINT_NAMES:
            self.assertAlmostEqual(current[name], stopped[name], delta=0.015, msg=name)

    def test_cancel_during_planning_and_reject_competing_goal(self):
        handle = self.send()
        self.wait_for(lambda: any(msg.phase == "planning" for msg in self.feedback))
        contender = self.wait(self.client.send_goal_async(PickPlace.Goal()))
        self.assertFalse(contender.accepted, "A second goal must not reset the active scene")
        self.cancel(handle)
        self.assertFalse(any(msg.phase == "executing" for msg in self.feedback))

    def test_invalid_motion_is_rejected(self):
        handle = self.wait(self.client.send_goal_async(PickPlace.Goal(
            use_default_motion=False, velocity_scale=0.0)))
        self.assertFalse(handle.accepted)

    def test_success_after_cancellation_reports_progress_and_result(self):
        handle = self.send()
        response = self.wait(handle.get_result_async(), 180)
        self.active = None
        self.assertEqual(response.status, GoalStatus.STATUS_SUCCEEDED, response.result.message)
        self.assertTrue(response.result.success)
        self.assertEqual(response.result.error_code, PickPlace.Result.SUCCESS)
        self.assertEqual(response.result.final_stage, "success")
        self.assertGreater(response.result.elapsed_seconds, 0)
        phases = {msg.phase for msg in self.feedback}
        self.assertTrue({"initializing", "planning", "executing", "cleanup"}.issubset(phases))
        for phase in ("planning", "executing"):
            stages = [msg for msg in self.feedback if msg.phase == phase]
            self.assertTrue(stages)
            self.assertTrue(all(msg.completed_stages <= msg.total_stages for msg in stages))
            counts = [msg.completed_stages for msg in stages]
            self.assertEqual(counts, sorted(counts))
