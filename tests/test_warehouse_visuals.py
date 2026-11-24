"""Factory rendering must follow physics rather than virtual planning poses."""

import copy
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from arm_moveit_config.warehouse_display import WarehouseDisplay
from arm_moveit_config.warehouse_scene import WarehouseScene
from arm_moveit_config.warehouse_visuals import environment_markers, factory_markers, part_markers


class FactoryVisualsTest(unittest.TestCase):
    def setUp(self):
        self.scene = WarehouseScene()
        self.poses = {item: Pose(position=Point(x=float(index), y=0.1, z=0.4),
                                 orientation=Quaternion(w=1.0))
                      for index, item in enumerate(self.scene.dynamic_ids)}

    def display(self):
        return SimpleNamespace(scene=self.scene, poses={}, colors=dict(self.scene.colors),
                               phase=None, last_parts=None, signage=Mock(), parts=Mock())

    def test_only_status_text_remains_and_old_labels_are_deleted(self):
        for phase in ("ready", "planning", "executing", "succeeded", "stopped"):
            markers = factory_markers(self.scene, phase).markers
            text = [m for m in markers if m.action == Marker.ADD and m.type == Marker.TEXT_VIEW_FACING]
            self.assertEqual([(m.id, m.text) for m in text], [(7, phase.upper())])
            self.assertEqual({m.id for m in markers if m.action == Marker.DELETE}, set(range(7)))

    def test_lamp_and_camera_are_not_drawn_twice(self):
        initial = self.scene.initial()
        excluded = {*self.scene.dynamic_ids, "tower_light_2"}
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


if __name__ == "__main__":
    unittest.main()
