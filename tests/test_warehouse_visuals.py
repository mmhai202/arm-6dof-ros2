"""Factory rendering must follow physics rather than virtual planning poses."""

import copy
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from rclpy.clock import ClockType
from rclpy.node import Node

from arm_moveit_config.warehouse_display import WarehouseDisplay
from arm_moveit_config.warehouse_scene import WarehouseScene
from arm_moveit_config.warehouse_task import WarehouseTask
from arm_moveit_config.warehouse_visuals import environment_markers, factory_markers, part_markers


class FactoryVisualsTest(unittest.TestCase):
    def setUp(self):
        self.scene = WarehouseScene()
        self.poses = {item: Pose(position=Point(x=float(index), y=0.1, z=0.4),
                                 orientation=Quaternion(w=1.0))
                      for index, item in enumerate(self.scene.dynamic_ids)}

    def display(self):
        display = SimpleNamespace(scene=self.scene, poses={}, colors=dict(self.scene.colors),
                                  wrist_pose=None, held_item=None,
                                  phase=None, last_parts=None, last_signage=None,
                                  environment_snapshot=environment_markers(self.scene),
                                  environment=Mock(), signage=Mock(), parts=Mock())
        display.publish_parts = lambda: WarehouseDisplay.publish_parts(display)
        return display

    def test_only_status_text_remains_and_old_labels_are_deleted(self):
        for phase in ("ready", "planning", "executing", "succeeded", "stopped"):
            markers = factory_markers(self.scene, phase).markers
            text = [m for m in markers if m.action == Marker.ADD and m.type == Marker.TEXT_VIEW_FACING]
            self.assertEqual([(m.id, m.text) for m in text], [(7, phase.upper())])
            self.assertEqual({m.id for m in markers if m.action == Marker.DELETE}, {*range(7), 8})
            self.assertFalse(any(m.action == Marker.ADD and m.type == Marker.CUBE for m in markers))

    def test_lamp_and_camera_are_not_drawn_twice(self):
        initial = self.scene.initial()
        excluded = set(self.scene.dynamic_ids)
        excluded.update(item.id for item in initial.object_colors if item.color.a == 0)
        expected = {index for index, obj in enumerate(initial.world.collision_objects) if obj.id not in excluded}
        self.assertEqual({marker.id for marker in environment_markers(self.scene).markers}, expected)

    def test_virtual_planning_attachments_and_resets_cannot_move_displayed_parts(self):
        before = part_markers(self.scene, self.poses, self.scene.colors)
        self.scene.select(self.scene.dynamic_ids[0], "order_A_0")
        self.scene.attach()
        self.scene.detach(self.scene.place_center)
        self.scene.centers[self.scene.object_id] = list(self.scene.place_center)
        self.scene.initial()
        self.assertEqual(part_markers(self.scene, self.poses, self.scene.colors), before)
        self.assertEqual(self.poses[self.scene.object_id].position.z, 0.4)

    def test_physics_reset_removes_missing_parts_without_inventing_poses(self):
        poses = dict(self.poses)
        del poses[self.scene.dynamic_ids[2]]
        markers = part_markers(self.scene, poses, self.scene.colors).markers
        self.assertEqual(len(markers), 6)
        self.assertEqual(markers[2].action, Marker.DELETE)
        self.assertEqual(sum(m.action == Marker.ADD for m in markers), 5)
        self.assertTrue(all(m.action == Marker.DELETE for m in part_markers(self.scene, poses, {}).markers))

    def test_repeated_progress_does_not_recreate_status_markers(self):
        display = self.display()
        for stage in ("home", "attach", "lift", "detach"):
            WarehouseDisplay.on_status(display, String(data=json.dumps({
                "phase": "planning", "stage": stage, "colors": self.scene.colors})))
        self.assertEqual(display.signage.publish.call_count, 1)
        WarehouseDisplay.on_status(display, String(data=json.dumps({
            "phase": "executing", "colors": self.scene.colors})))
        self.assertEqual(display.signage.publish.call_count, 2)

    def test_updates_preserve_all_six_parts_for_late_rviz_subscribers(self):
        display = self.display()
        names = [f"{item}::{item}" for item in self.scene.dynamic_ids]
        WarehouseDisplay.on_links(display, LinkStates(name=names, pose=list(self.poses.values())))
        # A physics update is displayed immediately, without waiting for a timer.
        self.assertEqual(display.parts.publish.call_count, 1)
        WarehouseDisplay.publish_parts(display)
        WarehouseDisplay.publish_parts(display)
        self.assertEqual(display.parts.publish.call_count, 1)
        changed = copy.deepcopy(list(self.poses.values()))
        changed[0].position.z += 0.1
        WarehouseDisplay.on_links(display, LinkStates(name=names, pose=changed))
        WarehouseDisplay.publish_parts(display)
        latest = display.parts.publish.call_args.args[0]
        self.assertEqual(len(latest.markers), 6)
        self.assertTrue(all(m.action == Marker.ADD for m in latest.markers))
        self.assertAlmostEqual(latest.markers[0].pose.position.z, 0.5)

    def test_reset_recovers_static_scene_without_new_physics_or_status(self):
        display = self.display()
        rendered = {}

        def receive(message):
            for marker in message.markers:
                key = marker.ns, marker.id
                if marker.action == Marker.ADD:
                    rendered[key] = copy.deepcopy(marker)
                elif marker.action == Marker.DELETE:
                    rendered.pop(key, None)

        for publisher in (display.environment, display.parts, display.signage):
            publisher.publish.side_effect = receive
        WarehouseDisplay.on_links(display, LinkStates(
            name=[f"{item}::{item}" for item in self.scene.dynamic_ids], pose=list(self.poses.values())))
        WarehouseDisplay.on_status(display, String(data=json.dumps({
            "phase": "ready", "colors": self.scene.colors})))
        WarehouseDisplay.refresh_markers(display)
        expected = copy.deepcopy(rendered)
        self.assertEqual(sum(ns == "factory_parts" for ns, _ in rendered), 6)
        self.assertTrue(any(ns == "factory_environment" for ns, _ in rendered))
        self.assertTrue(any(ns == "factory_signage" for ns, _ in rendered))
        for _ in range(2):
            # Reset clears local RViz visuals but leaves subscriptions alive.
            rendered.clear()
            WarehouseDisplay.refresh_markers(display)
            self.assertEqual(rendered, expected)

    def test_refresh_does_not_invent_parts_or_restore_removed_bodies(self):
        display = self.display()
        WarehouseDisplay.refresh_markers(display)
        display.parts.publish.assert_not_called()
        display.signage.publish.assert_not_called()
        WarehouseDisplay.on_links(display, LinkStates(
            name=[f"{item}::{item}" for item in self.scene.dynamic_ids], pose=list(self.poses.values())))
        WarehouseDisplay.on_links(display, LinkStates())
        WarehouseDisplay.refresh_markers(display)
        self.assertTrue(all(marker.action == Marker.DELETE
                            for marker in display.parts.publish.call_args.args[0].markers))

    def test_visual_refresh_timer_runs_when_simulation_clock_is_paused(self):
        with patch.object(Node, "__init__", return_value=None), \
                patch.object(WarehouseDisplay, "create_publisher"), \
                patch.object(WarehouseDisplay, "create_subscription"), \
                patch.object(WarehouseDisplay, "create_timer") as timer:
            display = WarehouseDisplay()
        period, callback = timer.call_args.args
        self.assertEqual(period, 1.0)
        self.assertEqual(callback, display.refresh_markers)
        self.assertEqual(timer.call_args.kwargs["clock"].clock_type, ClockType.STEADY_TIME)

    def test_held_part_follows_wrist_tf_with_measured_rotation_and_slip(self):
        item = self.scene.dynamic_ids[0]
        wrist = Pose(position=Point(x=1.0, y=2.0, z=3.0),
                     orientation=Quaternion(z=2**-0.5, w=2**-0.5))
        self.poses[item] = Pose(position=Point(x=1.0, y=2.1, z=3.02),
                                orientation=Quaternion(z=1.0, w=0.0))
        markers = part_markers(self.scene, self.poses, self.scene.colors,
                               held_item=item, wrist_pose=wrist).markers
        held = markers[0]
        self.assertEqual(held.header.frame_id, "link_6")
        self.assertTrue(held.frame_locked)
        self.assertAlmostEqual(held.pose.position.x, 0.1)
        self.assertAlmostEqual(held.pose.position.y, 0.0)
        self.assertAlmostEqual(held.pose.position.z, 0.02)
        self.assertAlmostEqual(held.pose.orientation.z, 2**-0.5)
        self.assertAlmostEqual(held.pose.orientation.w, 2**-0.5)
        self.assertTrue(all(m.header.frame_id == self.scene.frame and not m.frame_locked
                            for m in markers[1:]))
        self.poses[item].position.y += 0.005
        self.poses[item].orientation = copy.deepcopy(wrist.orientation)
        slipped = part_markers(self.scene, self.poses, self.scene.colors,
                               held_item=item, wrist_pose=wrist).markers[0]
        self.assertAlmostEqual(slipped.pose.position.x, 0.105)
        self.assertAlmostEqual(slipped.pose.orientation.z, 0.0)
        self.assertAlmostEqual(slipped.pose.orientation.w, 1.0)

    def test_release_and_missing_physics_stop_following_the_wrist(self):
        display = self.display()
        item = self.scene.dynamic_ids[0]
        names = [f"{item}::{item}", "arm_sim::link_6"]
        poses = [self.poses[item], Pose(orientation=Quaternion(w=1.0))]
        WarehouseDisplay.on_links(display, LinkStates(name=names, pose=poses))

        def status(held_item):
            WarehouseDisplay.on_status(display, String(data=json.dumps({
                "phase": "executing", "colors": self.scene.colors, "held_item": held_item})))

        status(item)
        held = display.parts.publish.call_args.args[0].markers[0]
        self.assertTrue(held.frame_locked)
        status(None)
        released = display.parts.publish.call_args.args[0].markers[0]
        self.assertEqual((released.ns, released.id), (held.ns, held.id))
        self.assertFalse(released.frame_locked)
        self.assertEqual(released.header.frame_id, self.scene.frame)
        self.assertEqual(released.pose, self.poses[item])
        status(item)
        WarehouseDisplay.on_links(display, LinkStates(name=names[:1], pose=poses[:1]))
        self.assertFalse(display.parts.publish.call_args.args[0].markers[0].frame_locked)
        WarehouseDisplay.on_links(display, LinkStates())
        self.assertTrue(all(m.action == Marker.DELETE
                            for m in display.parts.publish.call_args.args[0].markers))

    def test_only_verified_physical_holding_is_reported_to_the_display(self):
        task = SimpleNamespace(ownership=SimpleNamespace(held=True), scene=self.scene,
                               object_id=self.scene.object_id, phase="executing", stage="lift",
                               move_count=0, status=Mock())
        for holding, fault in ((False, None), (True, None), (True, "Lost finger contact")):
            task.holding, task.fault = holding, fault
            WarehouseTask.publish_status(task)
            status = json.loads(task.status.publish.call_args.args[0].data)
            self.assertEqual(status["held_item"], task.object_id if holding and not fault else None)


if __name__ == "__main__":
    unittest.main()
