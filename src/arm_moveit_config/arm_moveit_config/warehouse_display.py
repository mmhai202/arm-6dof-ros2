"""Render physical factory bodies without showing MoveIt's planning previews."""

import json

from gazebo_msgs.msg import LinkStates
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from .warehouse_scene import PALETTE, WarehouseScene
from .warehouse_visuals import environment_markers, factory_markers, part_markers


class WarehouseDisplay(Node):
    def __init__(self):
        super().__init__("warehouse_display")
        self.scene = WarehouseScene()
        self.poses, self.colors = {}, {}
        self.last_parts = None
        self.phase = None
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.environment = self.create_publisher(MarkerArray, "/warehouse/environment", retained)
        self.parts = self.create_publisher(MarkerArray, "/warehouse/parts", retained)
        self.signage = self.create_publisher(MarkerArray, "/warehouse/markers", retained)
        self.links = self.create_subscription(
            LinkStates, "/gazebo/link_states", self.on_links, qos_profile_sensor_data)
        self.status = self.create_subscription(String, "/warehouse/status", self.on_status, retained)
        self.environment.publish(environment_markers(self.scene))
        self.timer = self.create_timer(1 / 30, self.publish_parts)

    def on_links(self, message):
        links = dict(zip(message.name, message.pose))
        self.poses = {item: links[f"{item}::{item}"] for item in self.scene.dynamic_ids
                      if f"{item}::{item}" in links}

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            colors = status["colors"]
            phase = status["phase"]
            if not isinstance(phase, str) or not isinstance(colors, dict):
                return
            if set(colors) != set(self.scene.dynamic_ids) or any(color not in PALETTE for color in colors.values()):
                return
        except (ValueError, KeyError, TypeError):
            return
        self.colors = colors
        if phase != self.phase:
            self.signage.publish(factory_markers(self.scene, phase))
            self.phase = phase

    def publish_parts(self):
        markers = part_markers(self.scene, self.poses, self.colors)
        if markers != self.last_parts:
            self.parts.publish(markers)
            self.last_parts = markers


def main():
    rclpy.init()
    node = WarehouseDisplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # SIGINT can invalidate a subscription while the executor is taking it.
        if rclpy.ok(context=node.context):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
