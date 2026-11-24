"""Contact-only Gazebo grasping, with MoveIt used for collision-aware motion."""

import argparse
import copy
import json
import math
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from arm_control.trajectory import ACTION_NAME, trajectory_goal, wait_for_future
from arm_description import GRIPPER_JOINT_NAMES, JOINT_NAMES
from arm_description.model import RobotModel
from arm_description.resources import load_yaml
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from gazebo_msgs.msg import ContactsState, LinkStates
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .gazebo_scene import gazebo_models, payload_in_box
from .scene_ownership import SceneOwnership
from .shelf_scene import OBJECT_ID, ShelfScene, scene_diff
from .shelf_task import ShelfTask


def pose_transform(pose):
    """Convert a Gazebo world pose to a homogeneous transform."""
    p, q = pose.position, pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    transform = np.eye(4)
    transform[:3, :3] = [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]
    transform[:3, 3] = [p.x, p.y, p.z]
    return transform


class GazeboShelfTask(ShelfTask):
    def __init__(self):
        super().__init__(ShelfScene(load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml")))
        for parameter, key in (("payload_mass", "mass"), ("payload_friction", "friction"),
                               ("grip_compression", "grip_compression")):
            self.declare_parameter(parameter, self.scene.config["physics"][key])
            self.scene.config["physics"][key] = self.get_parameter(parameter).value
        self.models = gazebo_models(self.scene)
        self.physics = self.scene.config["physics"]
        self.payload_pose = None
        self.model_stamp = None
        self.payload_center = None
        self.tool_tilt = 0.0
        self.world_links = ()
        self.link_generation = 0
        # Gazebo merges the fixed TCP/palm links into link_6. Derive that
        # constant offset from the URDF rather than duplicating tool dimensions.
        self.wrist_to_tcp = RobotModel.load(root_frame="link_6").tcp_transform([0.0] * len(JOINT_NAMES))
        self.last_contact = {side: -math.inf for side in ("left", "right")}
        self.contact_forces = {side: 0.0 for side in self.last_contact}
        self.holding = False
        self.grasp_center = None
        self.active_goal = None
        self.fault = None
        self.sync_enabled = False
        self.sync_future = None
        self.ownership = SceneOwnership()
        self.task_running = True
        self.stage = "initializing"
        self.report = self.create_publisher(String, "/shelf_grasp/status", 10)
        self.link_subscription = self.create_subscription(
            LinkStates, "/gazebo/link_states", self.on_links, qos_profile_sensor_data)
        self.contact_subscriptions = [self.create_subscription(
            ContactsState, f"/gripper/{side}_contacts",
            lambda message, side=side: self.on_contact(side, message), qos_profile_sensor_data,
        ) for side in self.last_contact]
        self.gripper = ActionClient(self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self.arm = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self.sync_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.timer = self.create_timer(0.2, self.monitor)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_links(self, message):
        self.world_links = tuple(message.name)
        self.link_generation += 1
        wrist_name, payload_name = "arm_sim::link_6", f"{OBJECT_ID}::{OBJECT_ID}"
        if wrist_name not in message.name or payload_name not in message.name:
            return  # Keep the last timestamp so missing feedback expires.
        wrist = message.pose[message.name.index(wrist_name)]
        payload = message.pose[message.name.index(payload_name)]
        # Both poses come from the same physics update. Mixing model_states
        # with the latest joint_states makes sensor delay look like payload slip.
        tcp = pose_transform(wrist) @ self.wrist_to_tcp
        self.tool_tilt = math.acos(float(np.clip(-tcp[2, 2], -1.0, 1.0)))
        p = payload.position
        self.payload_center = tcp[:3, :3].T @ (np.array([p.x, p.y, p.z]) - tcp[:3, 3])
        self.payload_pose = payload
        self.model_stamp = self.now()

    def on_contact(self, side, message):
        samples = []
        for state in message.states:
            if any(name.split("::")[0] == OBJECT_ID
                   for name in (state.collision1_name, state.collision2_name)):
                f = state.total_wrench.force
                samples.append(math.sqrt(f.x*f.x + f.y*f.y + f.z*f.z))
        # The bumper batches several physics steps in one sensor message.
        # Averaging avoids multiplying the reported newtons by that batch size.
        force = sum(samples) / len(samples) if samples else 0.0
        self.contact_forces[side] = force
        if force >= self.physics["min_contact_force"]:
            self.last_contact[side] = self.now()

    def bilateral_contact(self):
        return all(self.now() - stamp < self.physics["contact_timeout"]
                   for stamp in self.last_contact.values())

    def measured_payload(self):
        if self.payload_pose is None:
            raise RuntimeError("No Gazebo payload pose received.")
        obj = self.scene.payload([0, 0, 0])
        obj.pose = copy.deepcopy(self.payload_pose)
        return obj

    def local_payload_center(self):
        if self.payload_center is None or self.now() - self.model_stamp > self.physics["contact_timeout"]:
            raise RuntimeError("Missing or stale synchronized wrist/payload feedback.")
        return self.payload_center.copy()

    def monitor(self):
        if not self.sync_enabled or self.payload_pose is None:
            return
        if not self.ownership.resume():
            return
        drift = None
        if self.holding and self.fault is None:
            if self.now() - self.model_stamp > self.physics["contact_timeout"]:
                self.fault = "Gazebo payload feedback stopped."
            elif not self.bilateral_contact():
                self.fault = "Lost bilateral finger contact; payload may have slipped."
            else:
                drift = np.linalg.norm(self.local_payload_center() - self.grasp_center)
                if drift > self.physics["max_grasp_drift"]:
                    self.fault = f"Payload slipped {drift:.3f} m relative to the TCP."
            if self.fault and self.active_goal is not None:
                self.active_goal.cancel_goal_async()
                self.hold_arm()
        if not self.holding and self.sync_future is None:
            # A dedicated asynchronous client avoids recursive spinning in timers.
            diff = scene_diff()
            diff.world.collision_objects = [self.measured_payload()]
            self.sync_future = self.sync_client.call_async(ApplyPlanningScene.Request(scene=diff))
            self.sync_future.add_done_callback(self.on_sync_done)
        p = self.payload_pose.position
        self.report.publish(String(data=json.dumps({
            "stage": self.stage, "holding": self.holding, "fault": self.fault,
            "payload": [p.x, p.y, p.z], "finger_forces": self.contact_forces,
            "grasp_drift": None if drift is None else float(drift),
            "tool_tilt": self.tool_tilt if self.holding else None,
        })))
        if not self.task_running and self.sync_future is None:
            self.ownership.release()

    def on_sync_done(self, future):
        if self.sync_future is not future:
            return
        try:
            if not future.result().success:
                self.fault = "MoveIt rejected payload synchronization."
        except Exception as error:
            self.fault = f"Payload synchronization failed: {error}"
        finally:
            self.sync_future = None
            if not self.task_running:
                self.ownership.release()

    def acquire_scene(self, timeout=10):
        deadline = time.monotonic() + timeout
        while not self.ownership.claim():
            if time.monotonic() >= deadline:
                raise RuntimeError("Another shelf task is busy or awaiting a scene update; retry after it finishes.")
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop_sync(self):
        self.sync_enabled = False
        future = self.sync_future
        if future is not None:
            # Keep ownership until MoveIt acknowledges the last write. A timeout
            # must not release the lock with an update still in flight.
            rclpy.spin_until_future_complete(self, future)
            if not future.done():
                raise RuntimeError("ROS stopped before the last scene update was acknowledged.")
            # The executor may have queued the done callback for a later spin.
            self.on_sync_done(future)

    def idle(self):
        self.stop_sync()
        self.task_running = False
        self.ownership.release()
        self.sync_enabled = True

    def hold_arm(self):
        """Preempt the owned controller motion even if MoveIt ignores cancellation."""
        if self.latest_joints is None or not self.arm.server_is_ready():
            return
        positions = dict(zip(self.latest_joints.name, self.latest_joints.position))
        goal = trajectory_goal([([positions[name] for name in JOINT_NAMES], 0.2)])
        goal.trajectory.points[0].velocities = [0.0] * len(JOINT_NAMES)
        self.stop_future = self.arm.send_goal_async(goal)

    def advance(self, seconds):
        target = self.now() + seconds
        deadline = time.monotonic() + max(30, seconds * 5)
        while self.now() < target and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.now() < target:
            raise RuntimeError("Gazebo simulation clock is not advancing.")
        if self.fault:
            raise RuntimeError(self.fault)

    def wait_simulation_result(self, future, timeout):
        """Allow slow physics to finish while detecting a stopped simulation clock."""
        previous = self.now()
        deadline = previous + timeout
        last_progress = time.monotonic()
        while not future.done():
            current = self.now()
            if current > previous:
                previous = current
                last_progress = time.monotonic()
            if current >= deadline:
                raise TimeoutError("Trajectory exceeded its simulation-time deadline.")
            if time.monotonic() - last_progress > 30:
                raise TimeoutError("Gazebo simulation clock stopped while executing a trajectory.")
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def wait_for_models(self, present, names=None):
        """Wait for a physics snapshot confirming the whole scene transition."""
        generation = self.link_generation
        expected = set(self.models if names is None else names)
        deadline = time.monotonic() + 10
        while True:
            found = {name for name in expected
                     if any(link.startswith(f"{name}::") for link in self.world_links)}
            ready = found == expected if present else not found
            if self.link_generation > generation and ready:
                return
            if time.monotonic() >= deadline:
                action = "creating" if present else "removing"
                raise RuntimeError(f"Gazebo did not confirm {action} the demo scene.")
            rclpy.spin_once(self, timeout_sec=0.05)

    def spawn_scene(self):
        # Confirm the scene itself, not just the factory's replies. Retry
        # setup only, before planning or motion.
        for attempt in range(3):
            try:
                self.reset_scene()
                return
            except RuntimeError as error:
                if attempt == 2:
                    raise
                self.get_logger().warning(f"Scene reset incomplete: {error}; retrying before motion.")

    def reset_scene(self):
        # Remove the payload before its supports, then finish all removals
        # before reusing model names. A service reply alone is not a scene snapshot.
        for name in reversed(self.models):
            state = self.call(GetEntityState, "/gazebo/get_entity_state", GetEntityState.Request(name=name))
            if state.success:
                result = self.call(DeleteEntity, "/delete_entity", DeleteEntity.Request(name=name))
                if not result.success:
                    raise RuntimeError(f"Cannot reset {name}: {result.status_message}")
        self.wait_for_models(present=False)
        self.payload_pose = self.payload_center = self.model_stamp = None
        self.last_contact = dict.fromkeys(self.last_contact, -math.inf)
        self.contact_forces = dict.fromkeys(self.contact_forces, 0.0)
        for name, xml in self.models.items():
            request = SpawnEntity.Request(name=name, xml=xml)
            request.initial_pose.orientation.w = 1.0
            if name == OBJECT_ID:
                p = request.initial_pose.position
                p.x, p.y, p.z = map(float, self.scene.pick_center)
            response = self.call(SpawnEntity, "/spawn_entity", request)
            if not response.success:
                raise RuntimeError(f"Cannot spawn {name}: {response.status_message}")
            # The factory can reply before the model's links are initialized.
            # Finish that insertion before sending the next factory request.
            self.wait_for_models(present=True, names=(name,))
        found = {name for name in self.models
                 if any(link.startswith(f"{name}::") for link in self.world_links)}
        if found != set(self.models):
            raise RuntimeError("Gazebo removed a model while creating the demo scene.")
        self.advance(self.physics["settle_time"])
        if self.payload_pose is None or self.now() - self.model_stamp > self.physics["contact_timeout"]:
            raise RuntimeError("Gazebo did not publish fresh feedback for the spawned payload.")
        p = self.payload_pose.position
        if np.linalg.norm(np.array([p.x, p.y, p.z]) - self.scene.pick_center) > self.physics["max_grasp_drift"]:
            raise RuntimeError("Spawned payload did not settle at the shelf pick position.")

    def squeeze(self):
        if not self.gripper.wait_for_server(timeout_sec=20):
            raise RuntimeError("Gripper controller is unavailable.")
        command = self.scene.gripper_closed - self.physics["grip_compression"]
        goal = trajectory_goal(
            [([command] * len(GRIPPER_JOINT_NAMES), 2)], joint_names=GRIPPER_JOINT_NAMES,
        )
        goal.trajectory.points[0].velocities = [0.0] * len(GRIPPER_JOINT_NAMES)
        # The object blocks closure. Keep the spring target past contact without
        # treating that deliberate position error as a failed trajectory.
        goal.goal_tolerance = [JointTolerance(
            name=name, position=self.physics["grip_compression"] + 0.004, velocity=0.02,
        ) for name in GRIPPER_JOINT_NAMES]
        handle = wait_for_future(self.gripper.send_goal_async(goal), 15, node=self)
        if handle is None or not handle.accepted:
            raise RuntimeError("Gripper rejected the squeeze command.")
        try:
            result = self.wait_simulation_result(handle.get_result_async(), 15)
        except (TimeoutError, KeyboardInterrupt):
            if rclpy.ok(context=self.context):
                wait_for_future(handle.cancel_goal_async(), 5, node=self)
            raise
        if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            raise RuntimeError(f"Gripper failed: {result.result.error_string}")
        self.advance(0.4)
        if not self.bilateral_contact():
            raise RuntimeError("Grasp rejected: both fingers must contact the payload with force.")

    def change_scene(self, name):
        if not self.sync_enabled:
            return super().change_scene(name)  # Virtual attachment during preflight only.
        if name == "attach":
            # Drain any world-object update before removing it for planning.
            self.stop_sync()
            if self.fault:
                raise RuntimeError(self.fault)
            if not self.bilateral_contact():
                raise RuntimeError("Cannot lift: bilateral payload contact is missing.")
            self.grasp_center = self.local_payload_center()
            diff = self.scene.attach()
            p = diff.robot_state.attached_collision_objects[0].object.primitive_poses[0].position
            p.x, p.y, p.z = map(float, self.grasp_center)
            # This attachment is collision metadata for MoveIt only. Gazebo still
            # has a free body with no joint, constraint, pose setter or teleport.
            self.holding = True
            self.apply(diff)
            self.sync_enabled = True
        elif name == "detach":
            self.holding = False
            diff = self.scene.detach([0, 0, 0])
            diff.world.collision_objects = [self.measured_payload()]
            self.apply(diff)
        else:
            super().change_scene(name)

    def execute(self, trajectory):
        if self.fault:
            raise RuntimeError(self.fault)
        # Preflight paths start at ideal endpoints. Let the physical PID settle
        # within MoveIt's 0.01 rad start check before submitting the next path.
        first = trajectory.joint_trajectory.points[0]
        names = trajectory.joint_trajectory.joint_names
        deadline, wall_deadline = self.now() + 10.0, time.monotonic() + 30.0
        while True:
            actual = dict(zip(self.latest_joints.name, self.latest_joints.position))
            if all(abs(actual[name] - target) < 0.008 for name, target in zip(names, first.positions)):
                break
            if self.fault:
                raise RuntimeError(self.fault)
            if self.now() >= deadline or time.monotonic() >= wall_deadline:
                raise RuntimeError("Robot did not settle at the next preflight trajectory start.")
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.executor_client.wait_for_server(timeout_sec=15):
            raise RuntimeError("MoveIt execution is unavailable.")
        handle = wait_for_future(self.executor_client.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory)), 15, node=self)
        if handle is None or not handle.accepted:
            raise RuntimeError("MoveIt rejected execution.")
        self.active_goal = handle
        try:
            duration = trajectory.joint_trajectory.points[-1].time_from_start
            result = self.wait_simulation_result(handle.get_result_async(), duration.sec + duration.nanosec / 1e9 + 30)
            if self.fault:
                raise RuntimeError(self.fault)
            if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"MoveIt execution failed: {result.result.error_code.val}.")
        except (TimeoutError, KeyboardInterrupt):
            if rclpy.ok(context=self.context):
                wait_for_future(handle.cancel_goal_async(), 5, node=self)
            raise
        finally:
            self.active_goal = None

    def run(self, mode):
        self.spawn_scene()
        self.apply(self.scene.initial())
        self.set_finger_contact(False)
        self.wait_for_joints()
        if mode == "scene":
            self.sync_enabled = True
            self.stage = "scene_ready"
            self.get_logger().info("Gazebo cabinet, dynamic payload and box ready; poses synchronized to MoveIt.")
            return
        planned = self.plan_task()
        self.get_logger().info("All physical pick-and-place stages planned successfully.")
        self.sync_enabled = True
        if mode == "plan":
            self.stage = "planned"
            return
        for name, trajectory in planned:
            self.stage = name
            self.get_logger().info(f"Executing physical stage: {name}")
            if name == "close_gripper":
                self.squeeze()
            elif trajectory is None:
                self.change_scene(name)
            else:
                self.execute(trajectory)
            if name == "lift":
                self.advance(0.3)
                if self.payload_pose.position.z < self.scene.pick_center[2] + self.scene.motion["lift"] * 0.6:
                    raise RuntimeError("Lift failed: Gazebo payload did not leave the shelf.")
                self.get_logger().info("Verified physical lift with bilateral contact and free payload motion.")
            if name == "release_gripper":
                self.advance(self.physics["settle_time"])
                if not payload_in_box(self.scene, self.payload_pose.position):
                    raise RuntimeError("Release failed: physical payload did not settle inside the box.")
        self.stage = "success"
        self.monitor()
        self.get_logger().info("SUCCESS: contact-only grasp lifted the free Gazebo payload and released it inside the box.")

    def retain_after_failure(self, error):
        self.fault = str(error)
        self.stage = "failed"
        self.holding = False
        self.sync_enabled = True
        if self.payload_pose is not None:
            diff = self.scene.detach([0, 0, 0])
            diff.world.collision_objects = [self.measured_payload()]
            self.apply(diff)
        self.monitor()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("scene", "plan", "execute"))
    parser.add_argument("--once", action="store_true", help="Exit after the task instead of maintaining scene synchronization.")
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = GazeboShelfTask()
    result = 0
    try:
        # Rejected contenders must never reset models or retain/modify the scene.
        node.acquire_scene()
        try:
            node.run(args.mode or node.get_parameter("mode").value)
        except (RuntimeError, TimeoutError, ValueError) as error:
            if not rclpy.ok(context=node.context):
                return 0
            node.get_logger().error(f"Physical task stopped: {error}. Scene retained for inspection.")
            node.retain_after_failure(error)
            result = 1
        if not args.once:
            node.idle()
            while rclpy.ok(context=node.context) and not node.ownership.superseded:
                rclpy.spin_once(node, timeout_sec=0.2)
    except (RuntimeError, TimeoutError, ValueError) as error:
        node.get_logger().error(str(error))
        result = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok(context=node.context):
                node.stop_sync()
        finally:
            node.destroy_node()
            node.ownership.close()
            rclpy.try_shutdown()
    return result


if __name__ == "__main__":
    sys.exit(main())
