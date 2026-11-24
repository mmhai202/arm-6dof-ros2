"""Physically fulfill factory orders, replanning stock access after each move."""

import copy
import json
import math
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .gazebo_task import GazeboShelfTask
from .warehouse_observation import StockObservation
from .warehouse_plan import plan_orders
from .warehouse_scene import WarehouseScene


class WarehouseTask(GazeboShelfTask):
    def __init__(self, **kwargs):
        super().__init__(scene=WarehouseScene(), **kwargs)
        self.object_id = self.scene.object_id
        self.all_poses = {}
        self.stock_observation = None
        self.stock_result = None
        self.move_context = ""
        self.move_count = 0
        self.stock_subscription = self.create_subscription(
            String, "/warehouse/detections", self.on_stock, 10)
        self.status = self.create_publisher(String, "/warehouse/status",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_timer = self.create_timer(0.5, self.publish_status)

    def publish_status(self):
        if not self.ownership.held:
            return  # An idle scene must not overwrite the active task's progress.
        self.status.publish(String(data=json.dumps({
            "phase": self.phase, "stage": self.stage, "moves": self.move_count,
            "locations": self.scene.locations, "colors": self.scene.colors,
            "fault": self.fault,
        })))

    def report_progress(self, phase, stage, completed=0, total=0):
        if self.move_context and not stage.startswith(self.move_context):
            stage = f"{self.move_context} {stage}"
        super().report_progress(phase, stage, completed, total)
        self.publish_status()

    def on_links(self, message):
        # Physics is used to verify grip and landing, not to find stock to grasp.
        for item in self.scene.dynamic_ids:
            name = f"{item}::{item}"
            if name in message.name:
                index = message.name.index(name)
                self.all_poses[item] = (message.pose[index], self.now())
        super().on_links(message)

    def on_stock(self, message):
        if self.stock_observation is not None:
            try:
                self.stock_result = self.stock_observation.add(json.loads(message.data), self.now())
            except (TypeError, ValueError):
                self.stock_result = None

    def observe_stock(self):
        self.report_progress("perception", "scan_stock")
        self.stock_observation = StockObservation(self.scene, self.now())
        self.stock_result = None
        timeout = self.scene.config["perception"]["timeout"]
        deadline, wall_deadline = self.now() + timeout, time.monotonic() + max(45, timeout * 5)
        try:
            while self.now() < deadline and time.monotonic() < wall_deadline:
                self.check_cancel()
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.stock_result is not None:
                    for item, (color, center) in self.stock_result.items():
                        self.scene.colors[item] = color
                        self.scene.centers[item] = list(center)
                    return
            raise RuntimeError(f"Factory camera timed out: {self.stock_observation.reason}")
        finally:
            self.stock_observation = None

    def reset_scene(self):
        self.all_poses.clear()
        super().reset_scene()
        for item in self.scene.dynamic_ids:
            sample = self.all_poses.get(item)
            if sample is None or self.now() - sample[1] > self.physics["contact_timeout"]:
                raise RuntimeError(f"No fresh Gazebo feedback for {item}.")
            p = sample[0].position
            if np.linalg.norm(np.array([p.x, p.y, p.z]) - self.scene.centers[item]) > self.physics["max_grasp_drift"]:
                raise RuntimeError(f"Factory stock {item} did not settle on its deck.")

    def prepare_move(self, move):
        self.stop_sync()
        if move.source.startswith("stock_"):
            self.observe_stock()
        self.scene.select(move.item, move.destination)
        self.object_id = move.item
        self.payload_pose = self.payload_center = self.model_stamp = None
        self.grasp_verified = False
        self.camera_center = list(self.scene.pick_center)
        self.last_contact = dict.fromkeys(self.last_contact, -math.inf)
        self.contact_forces = dict.fromkeys(self.contact_forces, 0.0)
        self.apply(self.scene.initial())
        self.set_finger_contact(False)
        self.advance(0.15)

    def execute_move(self, planned):
        self.sync_enabled = True
        for index, (name, trajectory) in enumerate(planned):
            self.check_cancel()
            self.report_progress("executing", name, index, len(planned))
            if name == "close_gripper":
                self.squeeze()
            elif trajectory is None:
                self.change_scene(name)
            else:
                self.execute(trajectory)
            if name == "lift":
                self.advance(0.25)
                if self.payload_pose.position.z < self.scene.pick_center[2] + self.scene.motion["lift"] * 0.6:
                    raise RuntimeError("Factory lift failed: stock did not leave its support.")
            if name == "release_gripper":
                self.advance(self.physics["settle_time"])
                self.wait_for_placement()
                p = self.payload_pose.position
                target = self.scene.slot_centers[self.scene.destination]
                if np.linalg.norm(np.array([p.x, p.y]) - target[:2]) > self.scene.warehouse["placement_xy_tolerance"]:
                    raise RuntimeError("Stock landed outside its assigned order/buffer slot.")
                self.scene.centers[self.object_id] = [p.x, p.y, p.z]
                self.scene.orientations[self.object_id] = copy.deepcopy(self.payload_pose.orientation)
        self.stop_sync()

    def retain_after_failure(self, error):
        super().retain_after_failure(error)
        self.report_progress("stopped", "task_stopped")

    def run(self, mode):
        self.check_cancel()
        if not self.use_perception:
            raise ValueError("The factory task requires RGB-D perception.")
        self.report_progress("initializing", "reset_factory")
        seed = self.scene.config["perception"]["spawn_seed"]
        self.scene.randomize(seed if self.scene.config["perception"]["randomize_spawn"] else 0)
        self.spawn_center = list(self.scene.centers[self.object_id])
        self.spawn_scene()
        self.observe_stock()
        self.scene.pick_center = list(self.scene.centers[self.object_id])
        self.camera_center = list(self.scene.pick_center)
        self.apply(self.scene.initial())
        self.set_finger_contact(False)
        self.wait_for_joints()
        if mode == "scene":
            self.sync_enabled = True
            self.report_progress("ready", "factory_ready")
            self.get_logger().info("Factory ready: two parts per color, blue/red boxes and an open green tray.")
            return
        while True:
            self.check_cancel()
            moves = plan_orders(self.scene.locations, self.scene.colors, self.scene.requirements)
            if not moves:
                break
            move = moves[0]
            parts = move.destination.split("_")
            destination = (f"{self.scene.requirements[move.destination].title()} box, slot {int(parts[2]) + 1}"
                           if parts[0] == "order" else f"Green tray, slot {int(parts[1]) + 1}")
            self.move_context = (f"Move {self.move_count + 1}/{self.move_count + len(moves)} / "
                                 f"{self.scene.colors[move.item].upper()} to {destination} /")
            self.get_logger().info(f"Factory schedule ({len(moves)} moves remaining): {moves}")
            self.prepare_move(move)
            planned = self.plan_task()
            if mode == "plan":
                self.report_progress("planned", "next_move_ready", len(planned), len(planned))
                return
            if move.source.startswith("stock_"):
                planned_center = np.array(self.scene.pick_center)
                self.observe_stock()
                if np.linalg.norm(np.array(self.scene.centers[move.item]) - planned_center) > 0.006:
                    raise RuntimeError("Factory stock moved during planning; restart the task.")
            self.execute_move(planned)
            # Commit a discrete move only after the physical release is verified.
            self.scene.locations[move.item] = move.destination
            self.move_count += 1
            self.publish_status()
        self.move_context = ""
        self.sync_enabled = True
        self.report_progress("succeeded", "orders_complete", 6, 6)
        self.get_logger().info(f"SUCCESS: all six parts sorted by color after {self.move_count} physical moves.")
