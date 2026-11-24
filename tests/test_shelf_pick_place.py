"""Task geometry, collision-state transitions and incomplete-plan rejection."""

import copy
import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

try:
    from arm_moveit_config.shelf_scene import OBJECT_ID, ShelfScene
    from arm_moveit_config.shelf_task import ShelfTask, ensure_timing, nearest_angle, updated_state
    from moveit_msgs.msg import (
        AllowedCollisionEntry, AllowedCollisionMatrix, CollisionObject,
        MoveItErrorCodes, RobotState, RobotTrajectory,
    )
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from arm_description.resources import load_yaml, package_file
    from arm_description import GRIPPER_JOINT_NAMES, JOINT_NAMES
    from sensor_msgs.msg import JointState
    import xacro
    import numpy as np
    from arm_description.model import RobotModel
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


class ShelfSceneTest(unittest.TestCase):
    def setUp(self):
        self.scene = ShelfScene()

    def test_payload_is_on_shelf_and_place_fits_inside_open_box(self):
        s = self.scene
        half = s.config["object"]["size"][2] / 2
        self.assertAlmostEqual(s.pick_center[2] - half, s.config["shelf"]["surface_z"])
        b = s.config["box"]
        self.assertGreater(s.place_center[2] - half, b["thickness"])
        self.assertLess(s.place_center[2] + half, b["height"])
        for i in (0, 1):
            self.assertLess(abs(s.place_center[i] - b["center_xy"][i])
                            + s.config["object"]["size"][i] / 2,
                            b["size_xy"][i] / 2 - b["thickness"])
        names = {obj.id for obj in s.initial().world.collision_objects}
        self.assertTrue({"floor", "cabinet_shelf", "cabinet_top",
                         "box_bottom", "box_near", "box_far", "box_left", "box_right",
                         OBJECT_ID}.issubset(names))
        self.assertNotIn("box_top", names)
        self.assertNotIn("cabinet_back", names)

    def test_grasp_is_normalized_and_payload_offset_matches_pick_and_place(self):
        s = self.scene
        stages = {name: target for name, _, target in s.stages()}
        for name, center in (("grasp", s.pick_center), ("place", s.place_center)):
            p = stages[name]
            q = p.orientation
            self.assertAlmostEqual(sum(getattr(q, axis) ** 2 for axis in "xyzw"), 1)
            # A pi rotation about X maps the local payload +Z offset to world -Z.
            self.assertAlmostEqual(p.position.z - s.offset, center[2])
        self.assertLess(stages["leave_shelf"].position.x,
                        s.config["shelf"]["center_xy"][0] - s.config["shelf"]["size_xy"][0] / 2)

    def test_attach_and_detach_keep_collision_object_identity(self):
        s = self.scene
        attached = s.attach()
        self.assertTrue(attached.is_diff)
        self.assertTrue(attached.robot_state.is_diff)
        a = attached.robot_state.attached_collision_objects[0]
        self.assertEqual(a.object.id, OBJECT_ID)
        self.assertEqual(a.object.header.frame_id, "tcp")
        self.assertEqual(a.touch_links, ["finger_left", "finger_right"])
        # MoveIt automatically removes the world object when attaching it.
        self.assertEqual(attached.world.collision_objects, [])
        detached = s.detach(s.place_center)
        self.assertEqual(detached.robot_state.attached_collision_objects[0].object.operation,
                         CollisionObject.REMOVE)
        self.assertEqual(detached.world.collision_objects[0].id, OBJECT_ID)

    def test_oversize_payload_is_rejected(self):
        config = copy.deepcopy(self.scene.config)
        config["object"]["size"][0] = 0.5
        with self.assertRaisesRegex(ValueError, "fit inside"):
            ShelfScene(config)

    def test_invalid_motion_settings_are_rejected_before_planning(self):
        for key, values in (
            ("velocity_scale", (0.0, -0.1, 1.1, math.nan)),
            ("acceleration_scale", (0.0, 1.1, math.inf)),
            ("lift", (0.0, -0.1, math.inf)),
            ("withdraw_clearance", (-0.1, math.nan)),
            ("place_clearance", (-0.1, math.inf)),
        ):
            for value in values:
                with self.subTest(key=key, value=value):
                    config = copy.deepcopy(self.scene.config)
                    config["motion"][key] = value
                    with self.assertRaisesRegex(ValueError, f"motion.{key}"):
                        ShelfScene(config)

    def test_custom_motion_settings_update_waypoints_without_changing_fixtures(self):
        config = copy.deepcopy(self.scene.config)
        config["motion"].update(approach=0.12, lift=0.18, withdraw_clearance=0.22,
                                transfer_z=0.65, place_clearance=0.01)
        scene = ShelfScene(config)
        stages = {name: target for name, _, target in scene.stages()}
        grasp = stages["grasp"].position
        self.assertAlmostEqual(stages["pre_grasp"].position.z - grasp.z, 0.12)
        self.assertAlmostEqual(stages["lift"].position.z - grasp.z, 0.18)
        shelf_front = config["shelf"]["center_xy"][0] - config["shelf"]["size_xy"][0] / 2
        self.assertAlmostEqual(shelf_front - stages["leave_shelf"].position.x, 0.22)
        self.assertAlmostEqual(stages["above_box"].position.z, 0.65)
        payload_bottom = stages["place"].position.z - scene.offset - config["object"]["size"][2] / 2
        self.assertAlmostEqual(payload_bottom - config["box"]["thickness"], 0.01)
        self.assertEqual(scene.initial(), self.scene.initial())

    def test_visible_fingers_contact_cube_and_clear_supporting_shelf(self):
        root = ET.fromstring(xacro.process_file(str(package_file(
            "arm_description", "urdf/arm.urdf.xacro"))).toxml())
        palm_z = float(root.find("joint[@name='tcp_to_gripper_palm']/origin").get("xyz").split()[2])
        for side, sign in (("left", 1), ("right", -1)):
            link = root.find(f"link[@name='finger_{side}']")
            joint = root.find(f"joint[@name='finger_{side}_joint']")
            self.assertEqual(joint.get("type"), "prismatic")
            self.assertIsNotNone(link.find("visual/geometry"))
            thickness, _, length = map(float, link.find("collision/geometry/box").get("size").split())
            x = float(joint.find("origin").get("xyz").split()[0])
            inner_face = abs(x) + self.scene.gripper_closed - thickness / 2
            self.assertAlmostEqual(inner_face, self.scene.config["object"]["size"][0] / 2)
            axis = float(joint.find("axis").get("xyz").split()[0])
            self.assertEqual(axis, sign)
            finger_z = float(link.find("collision/origin").get("xyz").split()[2])
            tip_z = palm_z + finger_z + length / 2
            grasp = next(p for name, _, p in self.scene.stages() if name == "grasp")
            self.assertGreater(grasp.position.z - tip_z, self.scene.config["shelf"]["surface_z"])
            hardware = root.find(f"ros2_control/joint[@name='finger_{side}_joint']")
            self.assertIsNotNone(hardware.find("command_interface[@name='effort']"))

    def test_gripper_mount_is_concentric_with_actual_cad_face(self):
        raw = package_file("arm_description", "meshes/ur7e/link_6.stl").read_bytes()
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
        triangles = np.frombuffer(raw, dtype=dtype, offset=84)["vertices"]
        # Derive the mounting face from the CAD itself, independently of URDF offsets.
        face = triangles[np.all(np.abs(triangles[:, :, 2]) < 1e-6, axis=1)].reshape(-1, 3)
        self.assertGreater(len(face), 100)
        center = (face.min(axis=0) + face.max(axis=0)) / 2
        np.testing.assert_allclose(center, [0.0, 0.0, 0.0], atol=1e-6)
        transform = np.eye(4)
        for joint in RobotModel.load(root_frame="link_6", tip_frame="gripper_palm").chain:
            transform = transform @ joint.origin
            np.testing.assert_allclose(transform[:2, 3], center[:2], atol=1e-6)
            np.testing.assert_allclose(transform[:3, 2], [0.0, 0.0, 1.0], atol=1e-8)
        root = ET.fromstring(xacro.process_file(str(package_file(
            "arm_description", "urdf/arm.urdf.xacro"))).toxml())
        collision = root.find("link[@name='link_6']/collision")
        origin_z = float(collision.find("origin").get("xyz").split()[2])
        length = float(collision.find("geometry/cylinder").get("length"))
        self.assertLessEqual(origin_z + length / 2, center[2] + 1e-6)


class TaskFailureTest(unittest.TestCase):
    def test_payload_contact_rules_only_change_finger_pairs(self):
        matrix = AllowedCollisionMatrix(entry_names=["link_1", "link_2"], entry_values=[
            AllowedCollisionEntry(enabled=[False, True]),
            AllowedCollisionEntry(enabled=[True, False]),
        ])
        task = SimpleNamespace(call=Mock(return_value=SimpleNamespace(scene=SimpleNamespace(
            allowed_collision_matrix=matrix))), apply=Mock())
        for allowed in (True, False):
            ShelfTask.set_finger_contact(task, allowed)
            result = task.apply.call_args.args[0].allowed_collision_matrix
            self.assertTrue(result.entry_values[0].enabled[1])
            payload_index = result.entry_names.index(OBJECT_ID)
            for name in ("finger_left", "finger_right"):
                index = result.entry_names.index(name)
                self.assertEqual(result.entry_values[payload_index].enabled[index], allowed)
                self.assertEqual(result.entry_values[index].enabled[payload_index], allowed)
            self.assertFalse(result.entry_values[payload_index].enabled[0])

    def test_arm_motion_preserves_finger_state_and_gripper_motion_preserves_arm(self):
        names = [*JOINT_NAMES, *GRIPPER_JOINT_NAMES]
        state = RobotState(is_diff=True, joint_state=JointState(
            name=names, position=[0.0] * 6 + [0.025, 0.025]))
        arm_path = JointTrajectory(joint_names=list(JOINT_NAMES), points=[
            JointTrajectoryPoint(positions=[0.5] * 6)])
        state = updated_state(state, arm_path)
        gripper_path = JointTrajectory(joint_names=list(GRIPPER_JOINT_NAMES), points=[
            JointTrajectoryPoint(positions=[0.0, 0.0])])
        state = updated_state(state, gripper_path)
        self.assertEqual(dict(zip(state.joint_state.name, state.joint_state.position)),
                         dict(zip(names, [0.5] * 6 + [0.0, 0.0])))

    def test_gripper_closes_before_attachment_and_opens_before_retreat(self):
        names = [name for name, _, _ in ShelfScene().stages()]
        for first, second in (("grasp", "allow_finger_contact"),
                              ("allow_finger_contact", "close_gripper"),
                              ("close_gripper", "attach"), ("attach", "lift"),
                              ("place", "detach"), ("detach", "release_gripper"),
                              ("release_gripper", "forbid_finger_contact"),
                              ("forbid_finger_contact", "retreat")):
            self.assertLess(names.index(first), names.index(second))

    def test_approach_tries_another_ik_branch_after_blocked_transfer(self):
        state = RobotState(is_diff=True, joint_state=JointState(
            name=list(JOINT_NAMES), position=[0.0] * 6))
        trajectory = RobotTrajectory(joint_trajectory=JointTrajectory(
            joint_names=list(JOINT_NAMES), points=[JointTrajectoryPoint(positions=[0.1] * 6)]))
        task = SimpleNamespace(
            scene=ShelfScene(), model=RobotModel.load(), call=Mock(return_value=SimpleNamespace(
                error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS), solution=state)),
            plan_motion=Mock(side_effect=[trajectory, trajectory, RuntimeError("blocked"),
                                         trajectory, trajectory, trajectory]),
        )
        poses = {n: p for n, _, p in task.scene.stages()}
        result = ShelfTask.approach_motion(task, poses["grasp"],
                                          [poses["pre_grasp"], poses["outside_shelf"]], state)
        self.assertIs(result, trajectory)
        self.assertEqual([call.args[0] for call in task.plan_motion.call_args_list],
                         ["cartesian", "cartesian", "ompl"] * 2)

    def test_equivalent_ik_angles_stay_near_current_state_within_urdf_limits(self):
        limits = RobotModel.load().limits
        self.assertAlmostEqual(nearest_angle(-6.0, 0.0, limits["joint_1"]), 2 * math.pi - 6)
        self.assertEqual(nearest_angle(-3.0, 3.0, limits["joint_3"]), -3.0)

    def test_missing_cartesian_timing_is_repaired_within_motion_limits(self):
        positions = ([0.0, 0.0], [0.2, -0.1], [0.3, -0.15])
        trajectory = RobotTrajectory(joint_trajectory=JointTrajectory(
            joint_names=["joint_1", "joint_2"], points=[
                JointTrajectoryPoint(positions=p) for p in positions]))
        ensure_timing(trajectory, 0.15, 0.15)
        points = trajectory.joint_trajectory.points
        limits = load_yaml("arm_moveit_config", "config/joint_limits.yaml")["joint_limits"]
        self.assertEqual([list(p.positions) for p in points], list(map(list, positions)))
        for a, b in zip(points, points[1:]):
            dt = (b.time_from_start.sec - a.time_from_start.sec
                  + (b.time_from_start.nanosec - a.time_from_start.nanosec) / 1e9)
            self.assertGreater(dt, 0)
            for i, name in enumerate(trajectory.joint_trajectory.joint_names):
                delta = abs(b.positions[i] - a.positions[i])
                self.assertLessEqual(1.5 * delta / dt, limits[name]["max_velocity"] * 0.15 + 1e-8)
                self.assertLessEqual(6 * delta / dt ** 2, limits[name]["max_acceleration"] * 0.15 + 1e-8)
            self.assertEqual(list(b.velocities), [0.0, 0.0])

    def test_partial_cartesian_path_is_never_accepted(self):
        task = SimpleNamespace(scene=ShelfScene(), call=Mock(return_value=SimpleNamespace(
            error_code=MoveItErrorCodes(val=MoveItErrorCodes.SUCCESS), fraction=0.8)))
        task.scene.motion.update(velocity_scale=0.3, acceleration_scale=0.2)
        with self.assertRaisesRegex(RuntimeError, "incomplete"):
            ShelfTask.plan_motion(task, "cartesian", ShelfScene().stages()[1][2], RobotState())
        request = task.call.call_args.args[2]
        self.assertTrue(request.avoid_collisions)
        self.assertGreater(request.revolute_jump_threshold, 0)
        self.assertAlmostEqual(request.max_velocity_scaling_factor, 0.3)
        self.assertAlmostEqual(request.max_acceleration_scaling_factor, 0.2)

    def test_planning_uses_one_attempt_then_broadens_only_for_recoverable_errors(self):
        trajectory = RobotTrajectory(joint_trajectory=JointTrajectory(
            joint_names=list(JOINT_NAMES), points=[
                JointTrajectoryPoint(positions=[0.0] * len(JOINT_NAMES)),
                JointTrajectoryPoint(positions=[0.1] * len(JOINT_NAMES)),
            ]))
        success = MoveItErrorCodes.SUCCESS
        for codes, attempts in (
                ([success], [1]),
                ([MoveItErrorCodes.INVALID_MOTION_PLAN, success], [1, 5]),
                ([MoveItErrorCodes.PLANNING_FAILED, success], [1, 5]),
                ([MoveItErrorCodes.TIMED_OUT, success], [1, 5]),
                ([MoveItErrorCodes.INVALID_MOTION_PLAN] * 3, [1, 5, 5]),
                ([MoveItErrorCodes.INVALID_ROBOT_STATE], [1])):
            with self.subTest(codes=codes):
                seen = []
                responses = iter(codes)

                def reply(service, name, request):
                    plan = request.motion_plan_request
                    seen.append(plan.num_planning_attempts)
                    self.assertEqual(plan.allowed_planning_time, task.scene.motion["planning_time"])
                    return SimpleNamespace(motion_plan_response=SimpleNamespace(
                        error_code=MoveItErrorCodes(val=next(responses)), trajectory=trajectory))

                task = SimpleNamespace(scene=ShelfScene(), call=Mock(side_effect=reply),
                                       goal_constraints=lambda target, names: ShelfTask.goal_constraints(
                                           None, target, names))
                if codes[-1] == success:
                    self.assertIs(ShelfTask.plan_motion(task, "ompl", [0.1] * len(JOINT_NAMES),
                                                       RobotState()), trajectory)
                else:
                    with self.assertRaisesRegex(RuntimeError, "OMPL planning failed"):
                        ShelfTask.plan_motion(task, "ompl", [0.1] * len(JOINT_NAMES), RobotState())
                self.assertEqual(seen, attempts)

    def test_failed_preflight_restores_scene_and_sends_no_motion(self):
        task = SimpleNamespace(
            scene=ShelfScene(), state=Mock(return_value=RobotState()),
            get_logger=Mock(return_value=Mock()), apply=Mock(), execute=Mock(),
            check_cancel=Mock(), report_progress=Mock(),
            set_finger_contact=Mock(),
            plan_motion=Mock(side_effect=RuntimeError("unreachable")),
        )
        with self.assertRaisesRegex(RuntimeError, "unreachable"):
            ShelfTask.plan_task(task)
        task.execute.assert_not_called()
        restored = task.apply.call_args.args[0]
        payload = next(obj for obj in restored.world.collision_objects if obj.id == OBJECT_ID)
        self.assertAlmostEqual(payload.primitive_poses[0].position.z, task.scene.pick_center[2])

    def test_direct_cycle_plans_from_current_arm_pose_and_finishes_at_retreat(self):
        for via_home in (False, True):
            with self.subTest(via_home=via_home):
                current = RobotState(is_diff=True, joint_state=JointState(
                    name=list((*JOINT_NAMES, *GRIPPER_JOINT_NAMES)),
                    position=[0.3] * len(JOINT_NAMES) + [0.025] * len(GRIPPER_JOINT_NAMES)))

                def motion(kind, target, state):
                    names = GRIPPER_JOINT_NAMES if kind == "gripper" else JOINT_NAMES
                    positions = target if kind in ("gripper", "ompl") else [0.5] * len(names)
                    return RobotTrajectory(joint_trajectory=JointTrajectory(
                        joint_names=list(names), points=[JointTrajectoryPoint(positions=positions)]))

                task = SimpleNamespace(
                    scene=ShelfScene(), state=Mock(return_value=current),
                    get_logger=Mock(return_value=Mock()), apply=Mock(),
                    check_cancel=Mock(), report_progress=Mock(),
                    change_scene=Mock(), set_finger_contact=Mock(),
                    plan_motion=Mock(side_effect=motion),
                    approach_motion=Mock(return_value=motion("ompl", [0.5] * len(JOINT_NAMES), current)),
                )
                planned = ShelfTask.plan_task(task, **({} if via_home else {"via_home": False}))
                names = [name for name, _ in planned]
                self.assertEqual(names[0], "open_gripper")
                self.assertIn("release_gripper", names)
                self.assertEqual(names[-1], "return_home" if via_home else "retreat")
                self.assertEqual("home" in names, via_home)
                if not via_home:
                    start = task.approach_motion.call_args_list[0].args[2].joint_state
                    positions = dict(zip(start.name, start.position))
                    self.assertEqual([positions[name] for name in JOINT_NAMES], [0.3] * len(JOINT_NAMES))


if __name__ == "__main__":
    unittest.main()
