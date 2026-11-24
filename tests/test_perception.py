"""Image geometry, timestamp validation, and separation of sensing from GT."""

import copy
import math
import random
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import numpy as np
from arm_description.resources import load_yaml
from arm_perception.geometry import camera_sdf, locate_cuboid, quaternion_from_rpy, transform_matrix
from arm_perception.detector import RGBDDetector
from arm_perception.mounting import camera_fixtures
from arm_perception.visualization import camera_markers
from arm_moveit_config.gazebo_task import GazeboShelfTask
from arm_moveit_config.perception_input import DetectionWindow, sample_spawn_center
from arm_moveit_config.shelf_scene import ShelfScene
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


class CameraVisualRecoveryTest(unittest.TestCase):
    def test_camera_markers_recover_after_reset_without_respawning(self):
        detector = SimpleNamespace(
            config=load_yaml("arm_perception", "config/camera.yaml"), camera_visuals=None,
            camera_marker_pub=Mock(), spawn_timer=Mock(), static_tf=Mock(),
            get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=12))),
            get_logger=Mock(), status_pub=Mock(), spawn_client=Mock())
        detector.publish_camera_markers = lambda: RGBDDetector.publish_camera_markers(detector)
        detector.publish_camera_markers()
        detector.camera_marker_pub.publish.assert_not_called()
        future = SimpleNamespace(result=lambda: SimpleNamespace(success=True))
        RGBDDetector.camera_ready(detector, future)
        expected = camera_markers(detector.config)
        self.assertEqual(detector.camera_marker_pub.publish.call_args.args[0], expected)
        detector.camera_marker_pub.publish.reset_mock()
        for _ in range(2):
            # The periodic callback must resend even with no new image/clock data.
            detector.publish_camera_markers()
            self.assertEqual(detector.camera_marker_pub.publish.call_args.args[0], expected)
        self.assertEqual(detector.camera_marker_pub.publish.call_count, 2)
        detector.static_tf.sendTransform.assert_called_once()
        detector.spawn_client.call_async.assert_not_called()

    def test_failed_camera_spawn_does_not_publish_visuals(self):
        detector = SimpleNamespace(spawn_timer=Mock(), camera_visuals=None,
                                   camera_marker_pub=Mock(), get_logger=Mock(), status_pub=Mock())
        future = SimpleNamespace(result=lambda: SimpleNamespace(success=False, status_message="unavailable"))
        RGBDDetector.camera_ready(detector, future)
        RGBDDetector.publish_camera_markers(detector)
        self.assertIsNone(detector.camera_visuals)
        detector.camera_marker_pub.publish.assert_not_called()


class RGBDGeometryTest(unittest.TestCase):
    def setUp(self):
        self.config = load_yaml("arm_perception", "config/camera.yaml")
        c = self.config["camera"]
        self.matrix = (transform_matrix(c["position"], quaternion_from_rpy(*c["rpy"]))
                       @ transform_matrix([0, 0, 0], quaternion_from_rpy(-math.pi/2, 0, -math.pi/2)))
        self.width, self.height = c["width"], c["height"]
        focal = self.width / (2 * math.tan(c["horizontal_fov"]/2))
        self.k = np.array([[focal, 0, self.width/2], [0, focal, self.height/2], [0, 0, 1]])

    def render(self, center):
        """Raycast a cuboid into aligned RGB/depth, independent of detector math."""
        v, u = np.mgrid[:self.height, :self.width]
        rays = np.stack(((u-self.k[0, 2])/self.k[0, 0], (v-self.k[1, 2])/self.k[1, 1],
                         np.ones_like(u)), axis=-1) @ self.matrix[:3, :3].T
        origin = self.matrix[:3, 3]
        half = np.array(self.config["detection"]["object_size"])/2
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (np.array(center)-half-origin)/rays
            t2 = (np.array(center)+half-origin)/rays
        entry = np.minimum(t1, t2).max(axis=-1)
        exit = np.maximum(t1, t2).min(axis=-1)
        visible = (exit >= entry) & (entry > 0)
        rgb = np.full((self.height, self.width, 3), [148, 89, 41], dtype=np.uint8)
        rgb[visible] = [242, 51, 15]
        depth = np.full(visible.shape, np.nan, dtype=np.float32)
        depth[visible] = entry[visible]
        return rgb, depth

    def test_localizes_randomized_positions_from_images(self):
        for x, y in ((0.41, 0.22), (0.45, 0.28), (0.43, 0.25)):
            with self.subTest(x=x, y=y):
                center = [x, y, 0.44]
                rgb, depth = self.render(center)
                actual, mask, extents = locate_cuboid(rgb, depth, self.k, self.matrix, self.config["detection"])
                np.testing.assert_allclose(actual, center, atol=0.002)
                self.assertGreater(mask.sum(), 40)
                np.testing.assert_allclose(extents[:2], [0.06, 0.04], atol=0.008)
                self.assertLess(extents[2], 0.002)

    def test_rejects_missing_color_invalid_depth_and_incomplete_view(self):
        rgb, depth = self.render([0.43, 0.25, 0.44])
        cases = [(np.zeros_like(rgb), depth)]
        for invalid in (0.0, float("nan"), float("inf")):
            cases.append((rgb, np.full_like(depth, invalid)))
        # A partial top face cannot establish both complete planar bounds.
        camera_points_z = np.indices(depth.shape)[0]
        occluded = rgb.copy()
        visible_rows = np.nonzero(np.isfinite(depth))[0]
        occluded[camera_points_z > (visible_rows.min()+visible_rows.max())//2] = 0
        cases.append((occluded, depth))
        for color, distance in cases:
            with self.assertRaises(ValueError):
                locate_cuboid(color, distance, self.k, self.matrix, self.config["detection"])

    def test_rejects_mismatched_images_and_bad_calibration(self):
        rgb, depth = self.render([0.43, 0.25, 0.44])
        with self.assertRaisesRegex(ValueError, "aligned"):
            locate_cuboid(rgb, depth[:10], self.k, self.matrix, self.config["detection"])
        with self.assertRaisesRegex(ValueError, "intrinsics"):
            locate_cuboid(rgb, depth, np.zeros((3, 3)), self.matrix, self.config["detection"])

    def test_roof_does_not_occlude_any_spawn_region_top_corner(self):
        scene = ShelfScene()
        shelf = scene.config["shelf"]
        roof_front = shelf["center_xy"][0] - shelf["size_xy"][0]/2
        origin = np.array(self.config["camera"]["position"])
        half = np.array(scene.config["object"]["size"])/2
        bounds = scene.config["perception"]["spawn_xy_bounds"]
        for x in bounds[0]:
            for y in bounds[1]:
                for dx in (-half[0], half[0]):
                    for dy in (-half[1], half[1]):
                        target = np.array([x+dx, y+dy, shelf["surface_z"]+2*half[2]])
                        for z in (shelf["top_z"], shelf["top_z"]+shelf["thickness"]):
                            hit = origin + (target-origin) * ((z-origin[2])/(target[2]-origin[2]))
                            self.assertLess(hit[0], roof_front, "Sightline must pass in front of the roof")

    def test_mount_rests_on_roof_and_gazebo_collision_poses_match_moveit(self):
        scene = ShelfScene()
        scene.config["perception"]["enabled"] = True
        fixtures = camera_fixtures(self.config)
        plate = next(part for part in fixtures if part["name"].endswith("_base"))
        shelf = scene.config["shelf"]
        self.assertAlmostEqual(plate["center"][2]-plate["size"][2]/2,
                               shelf["top_z"]+shelf["thickness"])
        for axis in (0, 1):
            self.assertLessEqual(abs(plate["center"][axis]-shelf["center_xy"][axis])+plate["size"][axis]/2,
                                 shelf["size_xy"][axis]/2)
        camera_to_base = transform_matrix(self.config["camera"]["position"],
                                         quaternion_from_rpy(*self.config["camera"]["rpy"]))
        sdf = ET.fromstring(camera_sdf(self.config))
        planning = {obj.id: obj for obj in scene.initial().world.collision_objects}
        for fixture in fixtures:
            name = fixture["name"]
            collision = sdf.find(f".//collision[@name='{name}']")
            xyzrpy = list(map(float, collision.findtext("pose").split()))
            actual = camera_to_base @ transform_matrix(xyzrpy[:3], quaternion_from_rpy(*xyzrpy[3:]))
            p = planning[name].primitive_poses[0]
            expected = transform_matrix([p.position.x, p.position.y, p.position.z],
                                        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            np.testing.assert_allclose(actual, expected, atol=1e-12)
            np.testing.assert_allclose(list(map(float, collision.findtext("geometry/box/size").split())),
                                       planning[name].primitives[0].dimensions)


class CameraTargetTest(unittest.TestCase):
    def setUp(self):
        self.scene = ShelfScene()
        self.window = DetectionWindow(self.scene, after=10.0)

    def message(self, stamp=10.1, center=(0.43, 0.25, 0.44), frame="base_footprint"):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, center)
        msg.pose.orientation.w = 1.0
        return msg

    def test_requires_distinct_fresh_stable_samples(self):
        msg = self.message()
        for _ in range(5):
            self.assertIsNone(self.window.add(msg, 10.1))
        self.assertIsNone(self.window.add(self.message(10.2), 10.2))
        np.testing.assert_allclose(self.window.add(self.message(10.3), 10.3), self.scene.pick_center)

    def test_rejects_stale_future_wrong_frame_outside_and_nonfinite(self):
        for msg in (self.message(9.9), self.message(10.1), self.message(12.0),
                    self.message(11.1, frame="camera_optical_frame"),
                    self.message(11.1, center=(0.8, 0.25, 0.44)),
                    self.message(11.1, center=(0.43, 0.25, 0.1)),
                    self.message(11.1, center=(float("nan"), 0.25, 0.44))):
            self.assertIsNone(self.window.add(msg, 11.1))
            self.assertEqual(len(self.window.samples), 0)

    def test_moving_target_and_long_gaps_do_not_pass(self):
        for stamp, x in ((10.1, 0.41), (10.2, 0.43), (10.3, 0.45)):
            self.assertIsNone(self.window.add(self.message(stamp, (x, 0.25, 0.44)), stamp))
        self.assertIsNone(self.window.add(self.message(11.0), 11.0))
        self.assertEqual(len(self.window.samples), 1)

    def test_allows_small_clock_topic_delivery_skew(self):
        for stamp in (10.1, 10.2):
            self.assertIsNone(self.window.add(self.message(stamp), stamp - 0.05))
        self.assertIsNotNone(self.window.add(self.message(10.3), 10.25))

    def test_random_spawn_is_bounded_repeatable_and_does_not_change_grasp(self):
        rng = random.Random(17)
        before = copy.deepcopy(self.scene.pick_center)
        first = sample_spawn_center(self.scene, rng)
        second = sample_spawn_center(self.scene, rng)
        self.assertNotEqual(first, second)
        self.assertEqual(first, sample_spawn_center(self.scene, random.Random(17)))
        self.assertEqual(before, self.scene.pick_center)
        self.scene.config["perception"]["spawn_xy_bounds"][0] = [0.1, 0.5]
        with self.assertRaises(ValueError):
            sample_spawn_center(self.scene, rng)

    def test_pregrasp_collision_pose_uses_camera_even_when_gt_disagrees(self):
        self.scene.pick_center = [0.42, 0.27, 0.44]
        task = SimpleNamespace(use_perception=True, grasp_verified=False,
                               scene=self.scene, payload_pose=Mock(), camera_center=self.scene.pick_center)
        obj = GazeboShelfTask.measured_payload(task)
        p = obj.primitive_poses[0].position
        self.assertEqual([p.x, p.y, p.z], self.scene.pick_center)
        task.camera_center = None
        with self.assertRaisesRegex(RuntimeError, "No valid camera target"):
            GazeboShelfTask.measured_payload(task)

    def test_missing_camera_times_out_without_returning_spawn_or_gt_position(self):
        task = SimpleNamespace(scene=self.scene, now=Mock(side_effect=[10.0, 10.0, 30.0]),
                               report_progress=Mock(), check_cancel=Mock(), detected_center=None)
        with patch("arm_moveit_config.gazebo_task.rclpy.spin_once"):
            with self.assertRaisesRegex(RuntimeError, "Perception timed out"):
                GazeboShelfTask.observe_object(task)
        self.assertIsNone(task.observation)

    def test_recheck_preserves_confirmed_collision_target_while_waiting(self):
        task = SimpleNamespace(scene=self.scene, clock=10.0, camera_center=list(self.scene.pick_center),
                               use_perception=True, grasp_verified=False, report_progress=Mock(),
                               check_cancel=Mock(), detected_center=None)
        task.now = lambda: task.clock

        def spin(*args, **kwargs):
            # The scene timer can run while a new stable window is incomplete.
            obj = GazeboShelfTask.measured_payload(task)
            self.assertAlmostEqual(obj.primitive_poses[0].position.x, 0.43)
            task.clock += 13

        with patch("arm_moveit_config.gazebo_task.rclpy.spin_once", side_effect=spin):
            with self.assertRaisesRegex(RuntimeError, "Perception timed out"):
                GazeboShelfTask.observe_object(task)
        self.assertEqual(task.camera_center, self.scene.pick_center)


if __name__ == "__main__":
    unittest.main()
