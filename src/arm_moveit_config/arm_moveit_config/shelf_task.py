"""Shared MoveIt preflight planning for the physical shelf task."""

import math
import time

from arm_control.poses import load_named_poses
from arm_control.trajectory import wait_for_future
from arm_description import GRIPPER_JOINT_NAMES, JOINT_NAMES
from arm_description.model import RobotModel
from arm_description.resources import load_yaml
from arm_moveit_config.shelf_scene import OBJECT_ID, TOUCH_LINKS, ShelfScene, scene_diff
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    AllowedCollisionEntry, CollisionObject, Constraints, JointConstraint, MoveItErrorCodes,
    PlanningSceneComponents, RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene, GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from .task_state import TaskCanceled, TaskProgress


class ShelfTask(Node):
    def __init__(self, scene, *, use_global_arguments=True, node_name="shelf_pick_place"):
        super().__init__(node_name, use_global_arguments=use_global_arguments,
                         parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cancel_requested = lambda: False
        self.progress_callback = lambda progress: None
        self.phase = "initializing"
        self.stage = "initializing"
        for key, value in scene.motion.items():
            self.declare_parameter(f"motion.{key}", value)
        self.scene = ShelfScene({**scene.config, "motion": {
            key: self.get_parameter(f"motion.{key}").value for key in scene.motion
        }})
        self.model = RobotModel.load()
        self.service_clients = {}
        self.executor_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.latest_joints = None
        self.joint_subscription = self.create_subscription(
            JointState, "/joint_states", self.on_joints, 10)
        self.declare_parameter("mode", "execute")

    def check_cancel(self):
        if self.cancel_requested():
            raise TaskCanceled("Task canceled by request.")

    def report_progress(self, phase, stage, completed=0, total=0):
        self.phase, self.stage = phase, stage
        self.progress_callback(TaskProgress(phase, stage, completed, total))

    def on_joints(self, message):
        if set((*JOINT_NAMES, *GRIPPER_JOINT_NAMES)).issubset(message.name):
            self.latest_joints = message

    def wait_for_joints(self):
        deadline = time.monotonic() + 30
        while self.latest_joints is None and time.monotonic() < deadline:
            self.check_cancel()
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joints is None:
            raise RuntimeError("No complete /joint_states received from Gazebo.")

    def call(self, service, name, request):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(service, name)
        client = self.service_clients[name]
        planning = service in (GetMotionPlan, GetPositionIK, GetCartesianPath)
        deadline = time.monotonic() + 30
        while not client.wait_for_service(timeout_sec=0.2):
            if planning:
                self.check_cancel()
            if time.monotonic() >= deadline:
                raise RuntimeError(f"MoveIt service unavailable: {name}")
        # Planning requests can be abandoned. Scene writes must be acknowledged
        # before cancellation can release ownership or restore the virtual scene.
        return wait_for_future(client.call_async(request), 45, node=self,
                               cancel_check=self.check_cancel if planning else None)

    def apply(self, scene):
        if any(item.object.operation == CollisionObject.REMOVE
               for item in scene.robot_state.attached_collision_objects):
            request = GetPlanningScene.Request(components=PlanningSceneComponents(
                components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
            current = self.call(GetPlanningScene, "/get_planning_scene", request).scene
            attached = {item.object.id for item in current.robot_state.attached_collision_objects}
            scene.robot_state.attached_collision_objects = [
                item for item in scene.robot_state.attached_collision_objects
                if item.object.operation != CollisionObject.REMOVE or item.object.id in attached
            ]
        response = self.call(ApplyPlanningScene, "/apply_planning_scene",
                             ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise RuntimeError("MoveIt rejected a planning scene update.")

    def state(self):
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ROBOT_STATE))
        current = self.call(GetPlanningScene, "/get_planning_scene", request).scene.robot_state
        positions = dict(zip(current.joint_state.name, current.joint_state.position))
        if not set((*JOINT_NAMES, *GRIPPER_JOINT_NAMES)).issubset(positions):
            raise RuntimeError("MoveIt has not received all arm and gripper joint states.")
        return robot_state(positions)

    def goal_constraints(self, target, names=JOINT_NAMES):
        tolerance = 0.00005 if names == GRIPPER_JOINT_NAMES else 0.001
        return Constraints(joint_constraints=[JointConstraint(
            joint_name=name, position=value, tolerance_above=tolerance,
            tolerance_below=tolerance, weight=1.0,
        ) for name, value in zip(names, target)])

    def plan_motion(self, kind, target, state):
        m = self.scene.motion
        if kind == "cartesian":
            request = GetCartesianPath.Request(
                header=Header(frame_id=self.scene.frame), start_state=state,
                group_name="arm", link_name="tcp", waypoints=[target],
                max_step=m["cartesian_step"], jump_threshold=2.0,
                revolute_jump_threshold=0.4, avoid_collisions=True,
                max_velocity_scaling_factor=m["velocity_scale"],
                max_acceleration_scaling_factor=m["acceleration_scale"],
            )
            response = self.call(GetCartesianPath, "/compute_cartesian_path", request)
            if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 0.999:
                raise RuntimeError(f"Cartesian path incomplete: {response.fraction:.1%}, "
                                   f"MoveIt code {response.error_code.val}.")
            trajectory = response.solution
        else:
            request = GetMotionPlan.Request()
            plan = request.motion_plan_request
            plan.group_name = "gripper" if kind == "gripper" else "arm"
            plan.pipeline_id = "ompl"
            plan.start_state = state
            plan.num_planning_attempts = 5
            plan.allowed_planning_time = m["planning_time"]
            plan.max_velocity_scaling_factor = m["velocity_scale"]
            plan.max_acceleration_scaling_factor = m["acceleration_scale"]
            plan.goal_constraints = [self.goal_constraints(
                target, GRIPPER_JOINT_NAMES if kind == "gripper" else JOINT_NAMES)]
            for _ in range(3):
                response = self.call(GetMotionPlan, "/plan_kinematic_path", request).motion_plan_response
                # A randomized path may fail the pipeline's final collision
                # validation. Resample it; never execute that rejected path.
                if response.error_code.val != MoveItErrorCodes.INVALID_MOTION_PLAN:
                    break
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"OMPL planning failed: MoveIt code {response.error_code.val}.")
            trajectory = response.trajectory
        points = trajectory.joint_trajectory.points
        if not points:
            raise RuntimeError("MoveIt returned an empty trajectory.")
        ensure_timing(trajectory, m["velocity_scale"], m["acceleration_scale"])
        return trajectory

    def plan_task(self):
        state = self.state()
        home = load_named_poses()["home"]
        task_stages = self.scene.stages()
        poses = {name: target for name, _, target in task_stages}
        stages = [
            ("open_gripper", "gripper", (self.scene.motion["gripper_open"],) * len(GRIPPER_JOINT_NAMES)),
            ("home", "ompl", home), *task_stages, ("return_home", "ompl", home),
        ]
        planned = []
        try:
            for index, (name, kind, target) in enumerate(stages):
                self.check_cancel()
                self.report_progress("planning", name, index, len(stages))
                self.get_logger().info(f"Planning: {name}")
                if kind == "scene":
                    self.change_scene(name)
                    planned.append((name, None))
                    continue
                if name in ("outside_shelf", "above_box"):
                    endpoint = poses["grasp" if name == "outside_shelf" else "place"]
                    via = [poses["pre_grasp"], target] if name == "outside_shelf" else [target]
                    trajectory = self.approach_motion(endpoint, via, state)
                else:
                    trajectory = self.plan_motion(kind, target, state)
                planned.append((name, trajectory))
                state = updated_state(state, trajectory.joint_trajectory)
            self.report_progress("planning", "complete", len(stages), len(stages))
        finally:
            # Planning uses virtual joint states; restore the payload to the shelf
            # before any real execution or after a rejected partial plan.
            self.apply(self.scene.initial())
            self.set_finger_contact(False)
        return planned

    def set_finger_contact(self, allowed):
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        matrix = self.call(GetPlanningScene, "/get_planning_scene", request).scene.allowed_collision_matrix
        for name in [OBJECT_ID, *TOUCH_LINKS]:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for row in matrix.entry_values:
                    row.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(matrix.entry_names)))
        obj_index = matrix.entry_names.index(OBJECT_ID)
        for link in TOUCH_LINKS:
            index = matrix.entry_names.index(link)
            matrix.entry_values[obj_index].enabled[index] = allowed
            matrix.entry_values[index].enabled[obj_index] = allowed
        diff = scene_diff()
        diff.allowed_collision_matrix = matrix
        self.apply(diff)

    def change_scene(self, name):
        if name in ("allow_finger_contact", "forbid_finger_contact"):
            self.set_finger_contact(name == "allow_finger_contact")
        elif name == "attach":
            self.apply(self.scene.attach())
        elif name == "detach":
            self.apply(self.scene.detach(self.scene.place_center))
        else:
            raise ValueError(f"Unknown scene stage: {name}")

    def approach_motion(self, endpoint, via, state):
        """Choose an IK branch that also permits the entire vertical approach.

        A pose-only OMPL goal can select an elbow posture that reaches the top
        waypoint but collides with the shelf or box during the later descent.
        Solve at the bottom and trace the approach backward, then plan to the
        entrance joints. Try another IK branch if free-space planning fails.
        """
        seeds = [state]
        for shoulder in (-2.3, 0.8):
            for elbow in (-1.5, 1.5):
                for wrist in (-1.57, 1.57):
                    values = dict(zip(state.joint_state.name, state.joint_state.position))
                    values.update(zip(JOINT_NAMES, [shoulder, -1.8, elbow, -1.5, wrist, 0.0]))
                    seeds.append(robot_state(values))
        for seed in seeds:
            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name = "arm"
            ik.ik_link_name = "tcp"
            ik.robot_state = seed
            ik.pose_stamped = PoseStamped(header=Header(frame_id=self.scene.frame), pose=endpoint)
            ik.avoid_collisions = True
            ik.timeout.nanosec = 300_000_000
            result = self.call(GetPositionIK, "/compute_ik", request)
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            bottom = result.solution
            bottom.is_diff = True
            try:
                for waypoint in via:
                    path = self.plan_motion("cartesian", waypoint, bottom).joint_trajectory
                    bottom = updated_state(bottom, path)
                positions = dict(zip(path.joint_names, path.points[-1].positions))
                current = dict(zip(state.joint_state.name, state.joint_state.position))
                target = tuple(nearest_angle(positions[name], current[name], self.model.limits[name])
                               for name in JOINT_NAMES)
                return self.plan_motion("ompl", target, state)
            except RuntimeError:
                continue
        raise RuntimeError("No collision-free IK branch supports the complete approach.")


def robot_state(positions):
    return RobotState(is_diff=True, joint_state=JointState(
        name=list(positions), position=list(positions.values())))


def updated_state(state, trajectory):
    """Arm and finger trajectories update separate joints of one virtual state."""
    positions = dict(zip(state.joint_state.name, state.joint_state.position))
    positions.update(zip(trajectory.joint_names, trajectory.points[-1].positions))
    return robot_state(positions)


def nearest_angle(angle, reference, limits):
    """Avoid unnecessary full turns when IK returns an equivalent joint angle."""
    first = math.ceil((limits.lower - angle) / (2 * math.pi))
    last = math.floor((limits.upper - angle) / (2 * math.pi))
    candidates = [angle + turn * 2 * math.pi for turn in range(first, last + 1)]
    if not candidates:
        raise RuntimeError("IK returned an angle outside the robot joint limits.")
    return min(candidates, key=lambda candidate: abs(candidate - reference))


def ensure_timing(trajectory, velocity_scale, acceleration_scale):
    """Recover from Humble Cartesian TOTG failures without changing the path.

    The fallback uses cubic segments with zero endpoint velocities. Their peak
    speed is 1.5*dq/dt and peak acceleration is 6*dq/dt**2.
    """
    joint_path = trajectory.joint_trajectory
    points = joint_path.points
    times = [p.time_from_start.sec + p.time_from_start.nanosec / 1e9 for p in points]
    if len(points) > 1 and times[0] >= 0 and all(b > a for a, b in zip(times, times[1:])):
        return
    limits = load_yaml("arm_moveit_config", "config/joint_limits.yaml")["joint_limits"]
    elapsed = 0.0
    for index, point in enumerate(points):
        if index:
            delta = [abs(a - b) for a, b in zip(point.positions, points[index - 1].positions)]
            elapsed += max([0.03] + [max(
                1.5 * dq / (limits[name]["max_velocity"] * velocity_scale),
                math.sqrt(6 * dq / (limits[name]["max_acceleration"] * acceleration_scale)),
            ) for name, dq in zip(joint_path.joint_names, delta)])
        sec, nanosec = divmod(round(elapsed * 1e9), 1_000_000_000)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        point.velocities = [0.0] * len(joint_path.joint_names)
        point.accelerations = []
