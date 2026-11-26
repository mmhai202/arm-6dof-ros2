"""Run physical grasps against a disposable run.sh session.

ARM_RUN_GAZEBO_GRASP_TESTS=1 python -m unittest discover -s tests -p test_gazebo_grasp_integration.py -v
Start with plain Gazebo/MoveIt: these tests reset the demo fixtures/payload and
move the arm. Both terminals must use the same ROS_DOMAIN_ID.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


ENABLED = os.environ.get("ARM_RUN_GAZEBO_GRASP_TESTS") == "1"
if ENABLED:
    import rclpy
    from arm_description.resources import load_yaml
    from arm_moveit_config.gazebo_scene import ENVIRONMENT_ID, payload_in_box
    from arm_moveit_config.shelf_scene import OBJECT_ID, ShelfScene
    from gazebo_msgs.srv import GetEntityState
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from std_msgs.msg import String


@unittest.skipUnless(ENABLED, "Set ARM_RUN_GAZEBO_GRASP_TESTS=1 with plain Gazebo/MoveIt running.")
class PhysicalGraspIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("physical_grasp_integration")
        cls.history = []
        cls.subscription = cls.node.create_subscription(
            String, "/shelf_grasp/status", lambda msg: cls.history.append(json.loads(msg.data)), 100)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def call(self, kind, name, request):
        client = self.node.create_client(kind, name)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=20), name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=15)
            self.assertTrue(future.done(), name)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def run_task(self, *parameters, mode="execute"):
        self.history.clear()
        with tempfile.TemporaryDirectory(prefix="arm_grasp_test_") as directory:
            log = Path(directory) / "task.log"
            with log.open("w") as stream:
                command = ["ros2", "run", "arm_moveit_config", "shelf_pick_place.py", "--once", "--mode", mode,
                           "--ros-args", "-p", "use_sim_time:=true"]
                for parameter in parameters:
                    command.extend(["-p", parameter])
                process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic() + 180
                    while process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(self.node, timeout_sec=0.05)
                    self.assertIsNotNone(process.poll(), "Physical task timed out")
                    for _ in range(5):
                        rclpy.spin_once(self.node, timeout_sec=0.02)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
            return process.returncode, log.read_text()

    def test_idle_scene_owner_yields_to_a_second_preflight(self):
        self.history.clear()
        with tempfile.TemporaryDirectory(prefix="arm_handoff_test_") as directory:
            log = Path(directory) / "idle_owner.log"
            with log.open("w") as stream:
                process = subprocess.Popen(
                    ["ros2", "run", "arm_moveit_config", "shelf_pick_place.py", "--mode", "scene"],
                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
                )
                try:
                    deadline = time.monotonic() + 60
                    while (process.poll() is None and time.monotonic() < deadline
                           and not any(sample["stage"] == "scene_ready" for sample in self.history)):
                        rclpy.spin_once(self.node, timeout_sec=0.05)
                    self.assertTrue(any(sample["stage"] == "scene_ready" for sample in self.history), log.read_text())
                    result, planning_log = self.run_task(mode="plan")
                    self.assertEqual(result, 0, planning_log[-12000:])
                    self.assertIn("All physical pick-and-place stages planned successfully", planning_log)
                    self.assertEqual(process.wait(timeout=10), 0, log.read_text())
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)

    def test_contact_grasp_lifts_transfers_and_releases(self):
        result, log = self.run_task()
        self.assertEqual(result, 0, log[-12000:])
        self.assertIn("Verified physical lift", log)
        self.assertIn("SUCCESS: contact-only grasp", log)
        scene = ShelfScene(load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml"))
        lifted = [sample for sample in self.history if sample["holding"]
                  and sample["payload"][2] > scene.pick_center[2] + 0.08]
        self.assertTrue(lifted, "Payload must physically leave the shelf")
        self.assertTrue(any(all(force > 0.2 for force in sample["finger_forces"].values())
                            for sample in lifted))
        drift = [sample["grasp_drift"] for sample in self.history
                 if sample["holding"] and sample.get("grasp_drift") is not None]
        self.assertTrue(drift, "Slip monitoring must use synchronized wrist/payload feedback")
        self.assertLess(max(drift), scene.config["physics"]["max_grasp_drift"])
        tilts = [sample["tool_tilt"] for sample in self.history
                 if sample["holding"] and sample.get("tool_tilt") is not None]
        self.assertTrue(tilts, "Loaded transfers must report the measured tool orientation")
        before_release = [sample["payload"][2] for sample in self.history
                          if sample["stage"] == "place" and sample["holding"]]
        response = self.call(GetEntityState, "/gazebo/get_entity_state", GetEntityState.Request(name=OBJECT_ID))
        self.assertTrue(response.success)
        self.assertTrue(payload_in_box(scene, response.state.pose.position))
        self.assertLess(abs(response.state.twist.linear.z), 0.05)
        self.assertTrue(before_release)
        self.assertGreater(min(before_release) - response.state.pose.position.z, 0.005,
                           "Opening must let the free body drop onto the box bottom")

    def test_repeated_resets_keep_fixtures_and_payload_on_shelf(self):
        scene = ShelfScene(load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml"))
        for reset in range(3):
            with self.subTest(reset=reset):
                result, log = self.run_task(mode="scene")
                self.assertEqual(result, 0, log)
                fixture = self.call(GetEntityState, "/gazebo/get_entity_state",
                                    GetEntityState.Request(name=ENVIRONMENT_ID))
                payload = self.call(GetEntityState, "/gazebo/get_entity_state",
                                    GetEntityState.Request(name=OBJECT_ID))
                self.assertTrue(fixture.success, "Reset must not remove the freshly spawned supports")
                self.assertTrue(payload.success, "Reset must not remove the freshly spawned payload")
                for axis, expected in zip("xyz", scene.pick_center):
                    self.assertAlmostEqual(getattr(payload.state.pose.position, axis), expected, delta=0.002)

    def test_low_friction_grasp_fails_and_updates_free_object_pose(self):
        result, log = self.run_task("payload_friction:=0.001")
        self.assertNotEqual(result, 0, log)
        self.assertNotIn("SUCCESS: contact-only grasp", log)
        self.assertTrue(any(sample["fault"] and ("slipped" in sample["fault"] or "contact" in sample["fault"])
                            for sample in self.history), log[-12000:])
        response = self.call(GetPlanningScene, "/get_planning_scene", GetPlanningScene.Request(
            components=PlanningSceneComponents(components=(
                PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                PlanningSceneComponents.WORLD_OBJECT_GEOMETRY))))
        self.assertNotIn(OBJECT_ID, [obj.object.id for obj in response.scene.robot_state.attached_collision_objects])
        actual = self.call(GetEntityState, "/gazebo/get_entity_state", GetEntityState.Request(name=OBJECT_ID))
        self.assertTrue(actual.success)
        obj = next(item for item in response.scene.world.collision_objects if item.id == OBJECT_ID)
        # --once stops synchronization at exit; the free body may keep falling
        # afterward. Compare MoveIt's final snapshot with the last reported pose.
        for axis, expected in zip("xyz", self.history[-1]["payload"]):
            self.assertAlmostEqual(getattr(obj.pose.position, axis),
                                   expected, delta=0.01)
