"""Stock access search, multi-part perception and factory collision geometry."""

import copy
from collections import Counter
import math
import random
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from arm_description.resources import load_yaml
from arm_moveit_config.gazebo_scene import ENVIRONMENT_ID, gazebo_models, payload_in_box
from arm_moveit_config.warehouse_observation import StockObservation
from arm_moveit_config.warehouse_plan import plan_orders
from arm_moveit_config.warehouse_scene import PALETTE, WarehouseScene
from arm_perception.geometry import camera_sdf, quaternion_from_rpy, transform_matrix
from arm_perception.mounting import camera_fixtures
from arm_perception.warehouse_detector import WarehouseDetector, draw_stock_overlay, locate_stock
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, TransformStamped
from rclpy.clock import ClockType
from sensor_msgs.msg import CameraInfo


class StockPlannerTest(unittest.TestCase):
    def test_random_layouts_sort_two_parts_of_each_color_to_matching_destinations(self):
        for seed in range(20):
            scene = WarehouseScene()
            scene.randomize(seed)
            self.assertEqual(Counter(scene.colors.values()), {"red": 2, "blue": 2, "green": 2})
            moves = plan_orders(scene.locations, scene.colors, scene.requirements)
            self.assertEqual(len(moves), 6)
            self.assertEqual(sum(move.destination.startswith("buffer_") for move in moves), 2)
            for move in moves:
                self.assertEqual(scene.locations[move.item], move.source)
                self.assertNotIn(move.destination, scene.locations.values())
                if move.source.startswith("stock_") and move.source.endswith("_1"):
                    self.assertNotIn(move.source[:-1] + "0", scene.locations.values())
                if move.destination in scene.requirements:
                    self.assertEqual(scene.colors[move.item], scene.requirements[move.destination])
                else:
                    self.assertEqual(scene.colors[move.item], "green")
                scene.locations[move.item] = move.destination
            self.assertEqual(plan_orders(scene.locations, scene.colors, scene.requirements), [])

    def test_green_parts_are_sorted_even_after_both_boxes_are_full(self):
        scene = WarehouseScene()
        for slot, color in scene.requirements.items():
            item = next(item for item in scene.dynamic_ids
                        if scene.colors[item] == color and scene.locations[item].startswith("stock_"))
            scene.locations[item] = slot
        moves = plan_orders(scene.locations, scene.colors, scene.requirements)
        self.assertEqual(len(moves), 2)
        for move in moves:
            self.assertEqual(scene.colors[move.item], "green")
            self.assertTrue(move.destination.startswith("buffer_"))

    def test_color_sorting_handles_green_stock_in_any_fixture_row(self):
        for seed in range(12):
            scene = WarehouseScene()
            colors = ["red", "red", "blue", "blue", "green", "green"]
            random.Random(seed).shuffle(colors)
            scene.colors = dict(zip(scene.dynamic_ids, colors))
            moves = plan_orders(scene.locations, scene.colors, scene.requirements)
            self.assertEqual(len(moves), 6)
            for move in moves:
                expected = scene.requirements.get(move.destination, "green")
                self.assertEqual(scene.colors[move.item], expected)
                scene.locations[move.item] = move.destination
            self.assertEqual(plan_orders(scene.locations, scene.colors, scene.requirements), [])

    def test_unavailable_color_and_insufficient_buffer_fail(self):
        scene = WarehouseScene()
        for requirements, capacity in (({**scene.requirements, "order_C_0": "yellow"}, 2),
                                       (scene.requirements, 1)):
            with self.assertRaisesRegex(ValueError, "Cannot fulfill"):
                plan_orders(scene.locations, scene.colors, requirements, buffer_slots=capacity)

    def test_resume_from_buffer_after_relocation(self):
        scene = WarehouseScene()
        wanted = next(item for item in scene.dynamic_ids if scene.colors[item] == "red")
        scene.locations[wanted] = "buffer_0"
        moves = plan_orders(scene.locations, scene.colors, scene.requirements)
        self.assertTrue(any(move.item == wanted and move.source == "buffer_0" for move in moves))


class FactoryGeometryTest(unittest.TestCase):
    def test_camera_support_is_centered_connected_and_clear_of_housing(self):
        config = load_yaml("arm_perception", "config/warehouse_camera.yaml")
        fixtures = {part["name"].removeprefix("perception_camera_"): part
                    for part in camera_fixtures(config)}

        def bounds(name):
            part = fixtures[name]
            rotation = transform_matrix([0, 0, 0], quaternion_from_rpy(*part["rpy"]))[:3, :3]
            half = np.abs(rotation) @ np.array(part["size"]) / 2
            return np.array(part["center"]) - half, np.array(part["center"]) + half

        camera_x, camera_y, _ = config["camera"]["position"]
        for part in fixtures.values():
            self.assertAlmostEqual(part["center"][0], camera_x)
        for name in ("factory_camera_drop", "factory_camera_plate"):
            self.assertAlmostEqual(fixtures[name]["center"][1], camera_y)
        # Every face in the support chain meets without a gap or penetration.
        chain = ("housing", "factory_camera_plate", "factory_camera_drop", "factory_crossbar")
        for lower, upper in zip(chain, chain[1:]):
            self.assertAlmostEqual(bounds(lower)[1][2], bounds(upper)[0][2])
        self.assertAlmostEqual(bounds("factory_foot")[1][2], bounds("factory_mast")[0][2])
        self.assertAlmostEqual(bounds("factory_mast")[1][2], bounds("factory_crossbar")[1][2])
        bar_min, bar_max = bounds("factory_crossbar")
        for name in ("factory_camera_drop", "factory_mast"):
            lower, upper = bounds(name)
            self.assertTrue(np.all(lower[:2] >= bar_min[:2] - 1e-12))
            self.assertTrue(np.all(upper[:2] <= bar_max[:2] + 1e-12))

        # New support pieces must also reach both Gazebo and MoveIt's geometry.
        camera = config["camera"]
        base_from_camera = transform_matrix(camera["position"], quaternion_from_rpy(*camera["rpy"]))
        sdf = ET.fromstring(camera_sdf(config))
        planning = {obj.id: obj for obj in WarehouseScene().initial().world.collision_objects}
        for part in fixtures.values():
            name = part["name"]
            collision = sdf.find(f".//collision[@name='{name}']")
            xyzrpy = list(map(float, collision.findtext("pose").split()))
            world = base_from_camera @ transform_matrix(xyzrpy[:3], quaternion_from_rpy(*xyzrpy[3:]))
            np.testing.assert_allclose(world[:3, 3], part["center"], atol=1e-12)
            p = planning[name].primitive_poses[0].position
            np.testing.assert_allclose([p.x, p.y, p.z], part["center"], atol=1e-12)
            np.testing.assert_allclose(planning[name].primitives[0].dimensions,
                                       list(map(float, collision.findtext("geometry/box/size").split())))

    def test_grasp_and_release_preserve_the_active_parts_color(self):
        scene = WarehouseScene()
        for item in scene.dynamic_ids:
            scene.select(item, "buffer_0")
            for diff in (scene.attach(), scene.detach(scene.place_center)):
                self.assertEqual(diff.object_colors[0].id, item)
                color = diff.object_colors[0].color
                np.testing.assert_allclose([color.r, color.g, color.b],
                                           PALETTE[scene.colors[item]], atol=1e-6)

    def test_boxes_match_their_parts_and_green_tray_has_no_walls(self):
        scene = WarehouseScene()
        initial = scene.initial()
        colors = {item.id: [item.color.r, item.color.g, item.color.b] for item in initial.object_colors}
        for order in scene.warehouse["orders"]:
            self.assertEqual(len(set(order["colors"])), 1)
            np.testing.assert_allclose(colors[f"tray_{order['id']}_wall_x_1"],
                                       PALETTE[order["colors"][0]], atol=1e-6)
        names = {item.id for item in initial.world.collision_objects}
        self.assertTrue({"buffer_pad_0", "buffer_pad_1"}.issubset(names))
        self.assertFalse(any(name.startswith("buffer_wall") for name in names))

    def test_physics_and_planning_share_geometry_colors_and_six_free_bodies(self):
        scene = WarehouseScene()
        models = gazebo_models(scene)
        self.assertEqual(set(models), {ENVIRONMENT_ID, *scene.dynamic_ids})
        environment = ET.fromstring(models[ENVIRONMENT_ID])
        self.assertEqual(environment.find("model/static").text, "true")
        for item in scene.dynamic_ids:
            root = ET.fromstring(models[item])
            self.assertEqual(root.find("model/static").text, "false")
            self.assertEqual(root.findall(".//joint"), [])
            rgba = root.find(".//visual/material/diffuse").text.split()
            np.testing.assert_allclose(list(map(float, rgba[:3])), PALETTE[scene.colors[item]], atol=1e-6)
        for obj in scene.initial().world.collision_objects:
            if obj.id in scene.dynamic_ids or obj.id.startswith("perception_camera_"):
                continue
            link = environment.find(f"model/link[@name='{obj.id}']")
            self.assertIsNotNone(link, obj.id)
            np.testing.assert_allclose(list(map(float, link.find("collision/geometry/box/size").text.split())),
                                       obj.primitives[0].dimensions)

    def test_rear_extraction_intersects_front_stock_unless_front_is_removed(self):
        scene = WarehouseScene()
        rear = scene.dynamic_ids[1]
        scene.select(rear, "buffer_0")
        lift = next(target for name, _, target in scene.stages() if name == "lift")
        carried_bottom = lift.position.z - scene.offset - 0.02
        self.assertLess(carried_bottom, scene.centers[scene.dynamic_ids[0]][2] + 0.02)
        self.assertGreater(carried_bottom, scene.config["shelf"]["surface_z"])

    def test_target_acceptance_uses_raised_surface_and_active_item_identity(self):
        scene = WarehouseScene()
        scene.select(scene.dynamic_ids[3], "order_B_1")
        self.assertEqual(scene.attach().robot_state.attached_collision_objects[0].object.id, scene.dynamic_ids[3])
        center = scene.slot_centers["order_B_1"]
        self.assertTrue(payload_in_box(scene, Point(x=center[0], y=center[1], z=center[2])))
        self.assertFalse(payload_in_box(scene, Point(x=center[0], y=center[1], z=0.032)))


class StockObservationTest(unittest.TestCase):
    def setUp(self):
        self.scene = WarehouseScene()
        self.message = {"frame": self.scene.frame, "stamp": 10.1, "objects": [
            {"color": self.scene.colors[item], "center": self.scene.centers[item]}
            for item in reversed(self.scene.dynamic_ids)]}

    def test_association_uses_fixture_slots_and_requires_distinct_stable_frames(self):
        window = StockObservation(self.scene, after=10.0)
        self.assertIsNone(window.add(self.message, 10.1))
        self.assertIsNone(window.add(self.message, 10.1))
        self.message["stamp"] = 10.2
        self.assertIsNone(window.add(self.message, 10.2))
        self.message["stamp"] = 10.3
        result = window.add(self.message, 10.3)
        self.assertEqual(set(result), set(self.scene.dynamic_ids))
        for item, (color, center) in result.items():
            self.assertEqual(color, self.scene.colors[item])
            np.testing.assert_allclose(center, self.scene.centers[item])

    def test_missing_ambiguous_stale_and_nonfinite_detections_are_rejected(self):
        cases = []
        missing = copy.deepcopy(self.message)
        missing["objects"].pop()
        cases.append(missing)
        duplicate = copy.deepcopy(self.message)
        duplicate["objects"][0] = copy.deepcopy(duplicate["objects"][1])
        cases.append(duplicate)
        invalid = copy.deepcopy(self.message)
        invalid["objects"][0]["center"][0] = math.nan
        cases.append(invalid)
        cases.append({**self.message, "stamp": 8.0})
        cases.append({**self.message, "frame": "world"})
        for message in cases:
            window = StockObservation(self.scene, after=10.0)
            for stamp in (10.1, 10.2, 10.3):
                self.assertIsNone(window.add(message, stamp))


class WarehousePreviewTest(unittest.TestCase):
    def test_timer_uses_wall_time_even_when_simulation_is_paused(self):
        with patch("arm_perception.warehouse_detector.RGBDDetector.__init__", return_value=None), \
                patch.object(WarehouseDetector, "create_publisher"), \
                patch.object(WarehouseDetector, "create_timer") as timer, \
                patch("arm_perception.warehouse_detector.cv2.setNumThreads"):
            detector = WarehouseDetector()
        self.assertEqual(timer.call_args.args, (0.1, detector.publish_preview))
        self.assertEqual(timer.call_args.kwargs["clock"].clock_type, ClockType.STEADY_TIME)

    def test_empty_preview_never_repeats_an_old_image(self):
        detector = SimpleNamespace(preview_image=None, image_pub=Mock())
        WarehouseDetector.publish_preview(detector)
        detector.image_pub.publish.assert_not_called()

    def test_preview_keeps_only_newest_frame_with_capture_header_even_if_detection_fails(self):
        for error in (False, True):
            with self.subTest(detection_failure=error):
                bridge = CvBridge()
                transform = TransformStamped()
                transform.transform.rotation.w = 1.0
                detector = SimpleNamespace(
                    bridge=bridge, info=CameraInfo(width=320, height=240), preview_image=None,
                    config={"frame": "base_footprint", "detection": {}},
                    tf_buffer=Mock(lookup_transform=Mock(return_value=transform)),
                    stock_pub=Mock(), status_pub=Mock(), image_pub=Mock())
                for index in (1, 2):
                    pixels = np.full((240, 320, 3), index * 30, dtype=np.uint8)
                    rgb = bridge.cv2_to_imgmsg(pixels, encoding="rgb8")
                    rgb.header.frame_id = "camera_optical_frame"
                    rgb.header.stamp.sec, rgb.header.stamp.nanosec = 12, index * 100_000_000
                    depth = bridge.cv2_to_imgmsg(np.ones((240, 320), dtype=np.float32), encoding="32FC1")
                    depth.header = copy.deepcopy(rgb.header)
                    detector.info.header = rgb.header
                    with patch("arm_perception.warehouse_detector.locate_stock", return_value=[],
                               side_effect=ValueError("occluded") if error else None):
                        WarehouseDetector.on_images(detector, rgb, depth)
                detector.image_pub.publish.assert_not_called()
                WarehouseDetector.publish_preview(detector)
                output = detector.image_pub.publish.call_args.args[0]
                self.assertEqual(output.header, rgb.header)
                preview = bridge.imgmsg_to_cv2(output)
                np.testing.assert_array_equal(preview[20:], pixels[20:])
                self.assertFalse(np.array_equal(preview[:20], pixels[:20]))
                np.testing.assert_array_equal(bridge.imgmsg_to_cv2(rgb), pixels)
                self.assertIsNone(detector.preview_image)
                WarehouseDetector.publish_preview(detector)
                detector.image_pub.publish.assert_called_once()


class WarehouseOverlayTest(unittest.TestCase):
    def test_labels_match_boxes_and_base_frame_coordinates_in_millimetres(self):
        objects = [{"color": "red", "bounds": [50, 50, 70, 70],
                    "center": [0.4312, -0.025, 0.440]},
                   {"color": "blue", "bounds": [150, 50, 170, 70],
                    "center": [0.571, 0.180, 0.441]},
                   {"color": "red", "bounds": [50, 100, 70, 120],
                    "center": [0.432, 0.350, 0.440]}]
        image = np.full((240, 320, 3), 80, dtype=np.uint8)
        before = copy.deepcopy(objects)
        with patch("arm_perception.warehouse_detector.cv2.putText", wraps=cv2.putText) as draw:
            draw_stock_overlay(image, objects, "base_footprint")
        labels = [call.args[1] for call in draw.call_args_list]
        self.assertIn("DETECTED 3 | RED 2  BLUE 1  GREEN 0", labels)
        self.assertIn("XYZ (mm) / base_footprint", labels)
        for label in ("R1", "B1", "R2", "R1  431 -25 440", "B1  571 180 441", "R2  432 350 440"):
            self.assertIn(label, labels)
        self.assertEqual(objects, before)
        # Object interiors remain visible; annotation is on their boundaries.
        np.testing.assert_array_equal(image[60, 60], [80, 80, 80])

    def test_failed_detection_is_distinct_from_a_successful_empty_scene(self):
        for unavailable, expected in ((False, "DETECTED 0 | RED 0  BLUE 0  GREEN 0"),
                                      (True, "DETECTION UNAVAILABLE")):
            with self.subTest(unavailable=unavailable), \
                    patch("arm_perception.warehouse_detector.cv2.putText", wraps=cv2.putText) as draw:
                draw_stock_overlay(np.zeros((240, 320, 3), np.uint8), [],
                                   "base_footprint", unavailable=unavailable)
                self.assertEqual([call.args[1] for call in draw.call_args_list], [expected])


class MultiObjectImageTest(unittest.TestCase):
    def render(self, scene):
        config = load_yaml("arm_perception", "config/warehouse_camera.yaml")
        c = config["camera"]
        matrix = (transform_matrix(c["position"], quaternion_from_rpy(*c["rpy"]))
                  @ transform_matrix([0, 0, 0], quaternion_from_rpy(-math.pi/2, 0, -math.pi/2)))
        height, width = c["height"], c["width"]
        focal = width / (2 * math.tan(c["horizontal_fov"] / 2))
        k = np.array([[focal, 0, (width - 1)/2], [0, focal, (height - 1)/2], [0, 0, 1]])
        v, u = np.mgrid[:height, :width]
        rays = np.stack(((u-k[0, 2])/focal, (v-k[1, 2])/focal, np.ones_like(u)), axis=-1) @ matrix[:3, :3].T
        rgb = np.zeros((height, width, 3), np.uint8)
        depth = np.full((height, width), np.inf, np.float32)
        bounds = {}
        for item in scene.dynamic_ids:
            center = np.array(scene.centers[item])
            with np.errstate(divide="ignore", invalid="ignore"):
                near = (center - np.array([0.03, 0.02, 0.02]) - matrix[:3, 3]) / rays
                far = (center + np.array([0.03, 0.02, 0.02]) - matrix[:3, 3]) / rays
            entry, exit = np.minimum(near, far).max(axis=-1), np.maximum(near, far).min(axis=-1)
            visible = (exit >= entry) & (entry > 0) & (entry < depth)
            depth[visible] = entry[visible]
            rgb[visible] = np.array(PALETTE[scene.colors[item]]) * 255
            v, u = np.nonzero(visible)
            bounds[item] = [int(u.min()), int(v.min()), int(u.max()), int(v.max())]
        return rgb, depth, k, matrix, config["detection"], bounds

    def test_six_individual_image_fits_including_multiple_same_color_parts(self):
        scene = WarehouseScene()
        rgb, depth, k, matrix, config, bounds = self.render(scene)
        result = locate_stock(rgb, depth, k, matrix, config)
        self.assertEqual(len(result), 6)
        self.assertEqual(Counter(item["color"] for item in result), {"red": 2, "blue": 2, "green": 2})
        for item in scene.dynamic_ids:
            found = min(result, key=lambda detection: np.linalg.norm(np.array(detection["center"]) - scene.centers[item]))
            self.assertEqual(found["color"], scene.colors[item])
            np.testing.assert_allclose(found["center"], scene.centers[item], atol=0.003)
            self.assertEqual(found["bounds"], bounds[item])

    def test_cropped_fit_still_rejects_occlusion_and_invalid_depth(self):
        scene = WarehouseScene()
        rgb, depth, k, matrix, config, bounds = self.render(scene)
        x0, y0, x1, y1 = bounds[scene.dynamic_ids[0]]
        rgb[y0:y1 + 1, (x0 + x1) // 2:x1 + 1] = 0
        result = locate_stock(rgb, depth, k, matrix, config)
        self.assertEqual(len(result), 5)
        for item in scene.dynamic_ids[1:]:
            found = min(result, key=lambda detection: np.linalg.norm(
                np.array(detection["center"]) - scene.centers[item]))
            np.testing.assert_allclose(found["center"], scene.centers[item], atol=0.003)
        self.assertEqual(locate_stock(rgb, np.zeros_like(depth), k, matrix, config), [])

    def test_cropped_fit_rejects_misaligned_images_and_invalid_calibration(self):
        rgb, depth, k, matrix, config, _ = self.render(WarehouseScene())
        with self.assertRaisesRegex(ValueError, "aligned"):
            locate_stock(rgb, depth[:10], k, matrix, config)
        with self.assertRaisesRegex(ValueError, "intrinsics"):
            locate_stock(rgb, depth, np.zeros((3, 3)), matrix, config)
