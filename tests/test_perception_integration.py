"""ARM_RUN_PERCEPTION_TESTS=1 with a disposable --pick-place session.

Uses Gazebo state only to score camera accuracy and physical placement.
All task targets are obtained from the RGB-D topic by the worker itself.
"""

import ast
import os
from pathlib import Path
import random
import re
import signal
import subprocess
import tempfile
import time
import unittest


ENABLED = os.environ.get("ARM_RUN_PERCEPTION_TESTS") == "1"
if ENABLED:
    import rclpy
    from arm_moveit_config.gazebo_scene import payload_in_box
    from arm_moveit_config.shelf_scene import OBJECT_ID, ShelfScene
    from gazebo_msgs.srv import GetEntityState


@unittest.skipUnless(ENABLED, "Set ARM_RUN_PERCEPTION_TESTS=1 with --pick-place running.")
class PerceptionIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("perception_integration")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.try_shutdown()

    def run_task(self, seed, *, missing_camera=False):
        with tempfile.TemporaryDirectory(prefix="arm_perception_test_") as directory:
            log = Path(directory) / "task.log"
            with log.open("w") as stream:
                command = ["ros2", "run", "arm_moveit_config", "shelf_pick_place.py", "--once", "--mode", "execute",
                           "--ros-args", "-p", "use_perception:=true", "-p", f"perception_seed:={seed}"]
                if missing_camera:
                    command += ["-r", "/perception/object_pose:=/test_no_camera_pose"]
                process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    deadline = time.monotonic() + 240
                    while process.poll() is None and time.monotonic() < deadline:
                        rclpy.spin_once(self.node, timeout_sec=0.05)
                    self.assertIsNotNone(process.poll(), "Camera-driven task timed out")
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=15)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait(timeout=5)
            return process.returncode, log.read_text()

    def test_camera_localizes_and_picks_two_distinct_positions(self):
        scene = ShelfScene()
        centers = []
        for seed in (17, 29):
            with self.subTest(seed=seed):
                code, log = self.run_task(seed)
                self.assertEqual(code, 0, log[-14000:])
                targets = re.findall(r"Camera target in base_footprint: (\[[^\]]+\])", log)
                self.assertGreaterEqual(len(targets), 2, log)
                center = ast.literal_eval(targets[0])
                centers.append(center)
                rng = random.Random(seed)
                bounds = scene.config["perception"]["spawn_xy_bounds"]
                expected = [rng.uniform(*bounds[0]), rng.uniform(*bounds[1]), 0.44]
                for actual, target in zip(center, expected):
                    self.assertAlmostEqual(actual, target, delta=0.003)
                self.assertIn("Verified physical lift", log)
                self.assertIn("SUCCESS: contact-only grasp", log)
                client = self.node.create_client(GetEntityState, "/gazebo/get_entity_state")
                try:
                    self.assertTrue(client.wait_for_service(timeout_sec=10))
                    future = client.call_async(GetEntityState.Request(name=OBJECT_ID))
                    rclpy.spin_until_future_complete(self.node, future, timeout_sec=10)
                    self.assertTrue(future.done())
                    result = future.result()
                    self.assertTrue(result.success)
                    self.assertTrue(payload_in_box(scene, result.state.pose.position))
                finally:
                    self.node.destroy_client(client)
        if len(centers) == 2:
            self.assertGreater(sum(abs(a-b) for a, b in zip(*centers)), 0.005)

    def test_missing_camera_aborts_before_planning_or_motion(self):
        code, log = self.run_task(17, missing_camera=True)
        self.assertNotEqual(code, 0, log)
        self.assertIn("Perception timed out", log)
        self.assertNotIn("Executing physical stage", log)
        self.assertNotIn("All physical pick-and-place stages planned", log)
