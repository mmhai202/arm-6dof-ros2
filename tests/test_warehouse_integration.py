"""ARM_RUN_WAREHOUSE_TESTS=1 with a disposable --warehouse session running.

This resets the factory and physically fulfills both orders. Use the same
ROS_DOMAIN_ID as the launch and leave at least 15 minutes for slow renderers.
"""

import json
from collections import Counter
import os
import time
import unittest


ENABLED = os.environ.get("ARM_RUN_WAREHOUSE_TESTS") == "1"
if ENABLED:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from arm_task_interfaces.action import PickPlace
    from arm_moveit_config.warehouse_scene import PALETTE, WarehouseScene
    from gazebo_msgs.srv import GetEntityState
    from gazebo_msgs.msg import LinkStates
    from moveit_msgs.msg import PlanningSceneComponents
    from moveit_msgs.srv import GetPlanningScene
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray


@unittest.skipUnless(ENABLED, "Set ARM_RUN_WAREHOUSE_TESTS=1 with --warehouse running.")
class FactoryIntegrationTest(unittest.TestCase):
    def test_orders_and_buffer_relocations_are_physically_verified(self):
        self.run_orders()

    def test_full_speed_orders_keep_grip_during_wrist_rotation(self):
        self.run_orders(full_speed=True)

    def run_orders(self, *, full_speed=False):
        rclpy.init()
        node = rclpy.create_node("warehouse_integration")
        history = []
        grasps = []
        physical = {}
        last_parts = None
        visual_checks, missing_parts, planning_errors = [], [], []
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def on_links(message):
            physical.clear()
            physical.update(zip(message.name, message.pose))

        def check_visuals():
            if last_parts is None or not history or history[-1]["phase"] not in ("planning", "executing"):
                return
            visible = [marker for marker in last_parts.markers if marker.action == Marker.ADD]
            missing_parts.append(len(visible) != 6)
            visual_checks.append(history[-1]["phase"])
            # Planning may move virtual collision bodies, while the physical
            # stock and its displayed representation must stay together.
            if history[-1]["phase"] == "planning":
                for marker in visible:
                    pose = physical.get(f"stock_item_{marker.id}::stock_item_{marker.id}")
                    if pose is not None:
                        a, b = marker.pose.position, pose.position
                        planning_errors.append(((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)**0.5)

        def on_parts(message):
            nonlocal last_parts
            last_parts = message
            check_visuals()

        def on_status(message):
            history.append(json.loads(message.data))
            # Stationary bodies need no new marker messages during planning.
            # Validate the retained snapshot on progress updates as well.
            check_visuals()

        node.create_subscription(String, "/warehouse/status", on_status, 100)
        node.create_subscription(String, "/shelf_grasp/status",
                                 lambda msg: grasps.append(json.loads(msg.data)), 100)
        node.create_subscription(LinkStates, "/gazebo/link_states", on_links, qos_profile_sensor_data)
        node.create_subscription(MarkerArray, "/warehouse/parts", on_parts, retained)
        client = ActionClient(node, PickPlace, "/warehouse_order")
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=30))
            goal = PickPlace.Goal()
            motion = WarehouseScene().motion
            if full_speed:
                goal.use_default_motion = False
                for name in ("approach", "lift", "withdraw_clearance", "transfer_z", "place_clearance"):
                    setattr(goal, name, motion[name])
                goal.velocity_scale = goal.acceleration_scale = 1.0
            else:
                self.assertEqual((motion["velocity_scale"], motion["acceleration_scale"]), (0.80, 0.80))
            goal_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, goal_future, timeout_sec=30)
            self.assertTrue(goal_future.done())
            handle = goal_future.result()
            self.assertTrue(handle.accepted)
            result_future = handle.get_result_async()
            deadline = time.monotonic() + 900
            while not result_future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not result_future.done():
                canceled = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, canceled, timeout_sec=30)
                rclpy.spin_until_future_complete(node, result_future, timeout_sec=45)
                self.fail("Factory task timed out; cancellation requested.")
            result = result_future.result().result
            self.assertTrue(result.success, result.message)
            carried = [sample for sample in grasps if sample["holding"] and sample["grasp_drift"] is not None]
            self.assertTrue(carried)
            max_drift = max(sample["grasp_drift"] for sample in carried)
            self.assertLess(max_drift, 0.003)
            self.assertGreater(max(sample["tool_tilt"] for sample in carried), 0.5,
                               "Exercise tilted transfers, where ODE used to cut the jaw force.")
            node.get_logger().info(f"Warehouse grip: full_speed={full_speed}, max drift={max_drift * 1000:.3f} mm")
            self.assertIn("planning", visual_checks)
            self.assertIn("executing", visual_checks)
            self.assertFalse(any(missing_parts), "A displayed part disappeared during the task.")
            self.assertTrue(planning_errors)
            self.assertLess(max(planning_errors), 0.01, "RViz showed a virtual planning pose.")
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            final = next(sample for sample in reversed(history) if sample["phase"] == "succeeded")
            self.assertEqual(final["moves"], 6)
            self.assertEqual(Counter(final["colors"].values()), {"red": 2, "blue": 2, "green": 2})
            self.assertEqual(len(set(final["locations"].values())), 6)
            scene = WarehouseScene()
            scene_service = node.create_client(GetPlanningScene, "/get_planning_scene")
            self.assertTrue(scene_service.wait_for_service(timeout_sec=15))
            color_query = scene_service.call_async(GetPlanningScene.Request(
                components=PlanningSceneComponents(components=PlanningSceneComponents.OBJECT_COLORS)))
            rclpy.spin_until_future_complete(node, color_query, timeout_sec=15)
            self.assertTrue(color_query.done())
            rendered_colors = {item.id: item.color for item in color_query.result().scene.object_colors}
            service = node.create_client(GetEntityState, "/gazebo/get_entity_state")
            self.assertTrue(service.wait_for_service(timeout_sec=15))
            for item, location in final["locations"].items():
                rendered = rendered_colors[item]
                for actual, expected in zip((rendered.r, rendered.g, rendered.b), PALETTE[final["colors"][item]]):
                    self.assertAlmostEqual(actual, expected, delta=1e-6)
                if location in scene.requirements:
                    self.assertEqual(final["colors"][item], scene.requirements[location])
                else:
                    self.assertTrue(location.startswith("buffer_"))
                    self.assertEqual(final["colors"][item], "green")
                query = service.call_async(GetEntityState.Request(name=item))
                rclpy.spin_until_future_complete(node, query, timeout_sec=15)
                self.assertTrue(query.done())
                measured = query.result()
                self.assertTrue(measured.success)
                target = scene.slot_centers[location]
                p = measured.state.pose.position
                self.assertLess(((p.x-target[0])**2 + (p.y-target[1])**2)**0.5, 0.018)
                self.assertAlmostEqual(p.z, target[2], delta=0.008)
                self.assertLess(abs(measured.state.twist.linear.z), 0.01)
        finally:
            client.destroy()
            node.destroy_node()
            rclpy.try_shutdown()
