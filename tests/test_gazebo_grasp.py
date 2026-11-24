"""Physical scene geometry, contact gating, and slip cancellation regressions."""

import copy
from concurrent.futures import Future
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

try:
    import numpy as np
    import xacro
    from arm_control.trajectory import trajectory_goal
    from arm_description import GRIPPER_JOINT_NAMES, JOINT_NAMES
    from arm_description.resources import load_yaml, package_file
    from arm_moveit_config.gazebo_scene import ENVIRONMENT_ID, gazebo_models, payload_in_box
    from arm_moveit_config.gazebo_task import GazeboShelfTask, main
    from arm_moveit_config.scene_ownership import SceneOwnership
    from arm_moveit_config.shelf_scene import OBJECT_ID, ShelfScene
    from gazebo_msgs.msg import ContactsState, ContactState, LinkStates
    from gazebo_msgs.srv import DeleteEntity, GetEntityState, SpawnEntity
    from geometry_msgs.msg import Point, Pose, Vector3, Wrench
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import JointState
except ImportError as exc:
    raise unittest.SkipTest("Activate Conda and source install/setup.bash") from exc


class PhysicalSceneTest(unittest.TestCase):
    def setUp(self):
        self.scene = ShelfScene(load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml"))

    def test_dynamic_payload_has_mass_and_no_physical_attachment(self):
        models = gazebo_models(self.scene)
        root = ET.fromstring(models[OBJECT_ID])
        self.assertEqual(root.findtext("model/static"), "false")
        self.assertIsNone(root.find(".//joint"))
        self.assertIsNone(root.find(".//plugin"))
        self.assertEqual(root.findtext("model/link/pose"), "0 0 0 0 0 0")
        self.assertAlmostEqual(float(root.findtext("model/link/inertial/mass")), 0.1)
        self.assertGreater(self.scene.gripper_closed - self.scene.config["physics"]["grip_compression"], 0)

    def test_gazebo_fixture_geometry_matches_moveit(self):
        root = ET.fromstring(gazebo_models(self.scene)[ENVIRONMENT_ID])
        self.assertEqual(root.findtext("model/static"), "true")
        for obj in self.scene.initial().world.collision_objects:
            if obj.id == OBJECT_ID:
                continue
            link = root.find(f"model/link[@name='{obj.id}']")
            self.assertIsNotNone(link, obj.id)
            np.testing.assert_allclose(list(map(float, link.findtext("collision/geometry/box/size").split())),
                                       obj.primitives[0].dimensions)
            p = obj.primitive_poses[0].position
            np.testing.assert_allclose(list(map(float, link.findtext("pose").split()))[:3], [p.x, p.y, p.z])

    def test_contact_sensor_targets_converted_collision_name(self):
        root = ET.fromstring(xacro.process_file(str(package_file(
            "arm_description", "urdf/arm.urdf.xacro"))).toxml())
        for side in ("left", "right"):
            collision = root.find(f"link[@name='finger_{side}']/collision").get("name")
            sensor = root.find(f"gazebo[@reference='finger_{side}']/sensor")
            self.assertIsNotNone(sensor)
            self.assertEqual(sensor.findtext("contact/collision"), collision + "_collision")

    def test_payload_that_cannot_be_squeezed_is_rejected(self):
        config = copy.deepcopy(self.scene.config)
        config["object"]["size"][0] = 0.04
        with self.assertRaisesRegex(ValueError, "travel to squeeze"):
            gazebo_models(ShelfScene(config))

    def test_only_landed_payload_inside_box_counts_as_placed(self):
        p = Pose().position
        p.x, p.y = self.scene.config["box"]["center_xy"]
        p.z = self.scene.config["box"]["thickness"] + self.scene.config["object"]["size"][2] / 2
        self.assertTrue(payload_in_box(self.scene, p))
        p.z += 0.08
        self.assertFalse(payload_in_box(self.scene, p))
        p.z -= 0.08
        p.x += 0.2
        self.assertFalse(payload_in_box(self.scene, p))


class ControllerCommandTest(unittest.TestCase):
    def test_arm_goal_preserves_waypoints_and_normalizes_rounded_durations(self):
        positions = [0.1] * len(JOINT_NAMES)
        goal = trajectory_goal([(positions, 1.9999999996), (positions, 3.25)])
        self.assertEqual(goal.trajectory.joint_names, list(JOINT_NAMES))
        self.assertEqual([list(point.positions) for point in goal.trajectory.points], [positions, positions])
        self.assertEqual([(point.time_from_start.sec, point.time_from_start.nanosec)
                          for point in goal.trajectory.points], [(2, 0), (3, 250_000_000)])

    def test_hold_uses_measured_arm_positions_and_zero_velocity(self):
        task = SimpleNamespace(arm=Mock(), latest_joints=JointState(
            name=list(reversed(JOINT_NAMES)), position=[0.6, 0.5, 0.4, 0.3, 0.2, 0.1]))
        GazeboShelfTask.hold_arm(task)
        goal = task.arm.send_goal_async.call_args.args[0]
        self.assertEqual(goal.trajectory.joint_names, list(JOINT_NAMES))
        point = goal.trajectory.points[0]
        self.assertEqual(list(point.positions), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertEqual(list(point.velocities), [0.0] * len(JOINT_NAMES))
        self.assertEqual((point.time_from_start.sec, point.time_from_start.nanosec), (0, 200_000_000))

    def test_squeeze_targets_only_fingers_and_allows_contact_position_error(self):
        scene = ShelfScene()
        task = SimpleNamespace(
            scene=scene, physics=scene.config["physics"], gripper=Mock(),
            wait_simulation_result=Mock(return_value=SimpleNamespace(result=SimpleNamespace(error_code=0))),
            advance=Mock(), bilateral_contact=Mock(return_value=True),
        )
        with patch("arm_moveit_config.gazebo_task.wait_for_future", return_value=Mock(accepted=True)):
            GazeboShelfTask.squeeze(task)
        goal = task.gripper.send_goal_async.call_args.args[0]
        self.assertEqual(goal.trajectory.joint_names, list(GRIPPER_JOINT_NAMES))
        point = goal.trajectory.points[0]
        self.assertEqual(list(point.positions), [scene.gripper_closed - task.physics["grip_compression"]] * 2)
        self.assertEqual(list(point.velocities), [0.0, 0.0])
        self.assertEqual((point.time_from_start.sec, point.time_from_start.nanosec), (2, 0))
        self.assertEqual([tolerance.name for tolerance in goal.goal_tolerance], list(GRIPPER_JOINT_NAMES))
        for tolerance in goal.goal_tolerance:
            self.assertAlmostEqual(tolerance.position, task.physics["grip_compression"] + 0.004)
            self.assertEqual(tolerance.velocity, 0.02)


class ShutdownTest(unittest.TestCase):
    def test_superseded_idle_process_exits_and_closes_ownership(self):
        with patch("arm_moveit_config.gazebo_task.rclpy") as ros, \
                patch("arm_moveit_config.gazebo_task.GazeboShelfTask") as task:
            node = task.return_value
            node.ownership.superseded = False
            ros.spin_once.side_effect = lambda *args, **kwargs: setattr(node.ownership, "superseded", True)
            self.assertEqual(main(["--mode", "scene"]), 0)
            node.idle.assert_called_once()
            node.destroy_node.assert_called_once()
            node.ownership.close.assert_called_once()

    def test_busy_owner_does_not_reset_or_retain_the_scene(self):
        with patch("arm_moveit_config.gazebo_task.rclpy"), \
                patch("arm_moveit_config.gazebo_task.GazeboShelfTask") as task:
            task.return_value.acquire_scene.side_effect = RuntimeError("Another shelf task is busy")
            self.assertEqual(main(["--once"]), 1)
            task.return_value.run.assert_not_called()
            task.return_value.retain_after_failure.assert_not_called()
            task.return_value.ownership.close.assert_called_once()

    def test_shutdown_during_task_does_not_try_to_retain_scene(self):
        for error in (KeyboardInterrupt(), ExternalShutdownException(), TimeoutError("ROS stopped")):
            with self.subTest(error=type(error).__name__), \
                    patch("arm_moveit_config.gazebo_task.rclpy") as ros, \
                    patch("arm_moveit_config.gazebo_task.GazeboShelfTask") as task:
                ros.ok.return_value = False
                task.return_value.run.side_effect = error
                self.assertEqual(main(["--mode", "execute", "--once"]), 0)
                task.return_value.retain_after_failure.assert_not_called()
                task.return_value.destroy_node.assert_called_once()
                ros.try_shutdown.assert_called_once()


class SceneResetTest(unittest.TestCase):
    def task(self, *, landed=True):
        scene = ShelfScene()
        events = []
        task = SimpleNamespace(
            scene=scene, models={ENVIRONMENT_ID: "fixture XML", OBJECT_ID: "payload XML"},
            physics=scene.config["physics"], now=Mock(return_value=10.0), advance=Mock(),
            payload_pose=Pose(), payload_center=np.zeros(3), model_stamp=9.0,
            last_contact={"left": 9.9, "right": 9.9}, contact_forces={"left": 5.0, "right": 5.0},
        )

        def call(service, name, request):
            events.append((service, request.name))
            return SimpleNamespace(success=True)

        def wait(present, names=None):
            events.append(("feedback", (present, names)))
            if present and names == (OBJECT_ID,):
                self.assertIsNone(task.payload_pose, "A previous payload pose must not satisfy respawn")
                self.assertTrue(all(stamp == -float("inf") for stamp in task.last_contact.values()))
                x, y, z = scene.pick_center
                task.payload_pose = Pose(position=Point(x=x, y=y, z=z if landed else 0.02))
                task.model_stamp = task.now()

        task.call = call
        task.wait_for_models = wait
        task.world_links = tuple(f"{name}::link" for name in task.models)
        task.reset_scene = lambda: GazeboShelfTask.reset_scene(task)
        task.get_logger = Mock()
        return task, events

    def test_reset_removes_payload_before_supports_and_confirms_empty_scene_before_respawn(self):
        task, events = self.task()
        GazeboShelfTask.spawn_scene(task)
        deletes = [name for service, name in events if service is DeleteEntity]
        self.assertEqual(deletes, [OBJECT_ID, ENVIRONMENT_ID])
        self.assertLess(events.index((DeleteEntity, ENVIRONMENT_ID)), events.index(("feedback", (False, None))))
        self.assertLess(events.index(("feedback", (False, None))), events.index((SpawnEntity, ENVIRONMENT_ID)))
        self.assertLess(events.index(("feedback", (True, (ENVIRONMENT_ID,)))), events.index((SpawnEntity, OBJECT_ID)))
        self.assertLess(events.index((SpawnEntity, OBJECT_ID)), events.index(("feedback", (True, (OBJECT_ID,)))))
        task.advance.assert_called_once_with(task.physics["settle_time"])

    def test_payload_on_floor_cannot_pass_scene_setup(self):
        task, _ = self.task(landed=False)
        with self.assertRaisesRegex(RuntimeError, "shelf pick position"):
            GazeboShelfTask.spawn_scene(task)

    def test_incomplete_setup_is_retried_before_any_motion(self):
        task = SimpleNamespace(reset_scene=Mock(side_effect=[RuntimeError("model disappeared"), None]),
                               get_logger=Mock())
        GazeboShelfTask.spawn_scene(task)
        self.assertEqual(task.reset_scene.call_count, 2)
        task.get_logger.return_value.warning.assert_called_once()

    def test_scene_barrier_waits_for_a_new_complete_physics_snapshot(self):
        task = SimpleNamespace(models={ENVIRONMENT_ID: "", OBJECT_ID: ""},
                               world_links=(), link_generation=1)
        snapshots = [(f"{ENVIRONMENT_ID}::shelf",),
                     (f"{ENVIRONMENT_ID}::shelf", f"{OBJECT_ID}::{OBJECT_ID}")]

        def receive(*args, **kwargs):
            task.world_links = snapshots.pop(0)
            task.link_generation += 1

        with patch("arm_moveit_config.gazebo_task.rclpy.spin_once", side_effect=receive) as spin:
            GazeboShelfTask.wait_for_models(task, present=True)
            self.assertEqual(spin.call_count, 2)
        task.world_links = ()
        with patch("arm_moveit_config.gazebo_task.rclpy.spin_once", side_effect=receive), \
                patch("arm_moveit_config.gazebo_task.time.monotonic", side_effect=[0.0, 11.0]):
            with self.assertRaisesRegex(RuntimeError, "confirm removing"):
                GazeboShelfTask.wait_for_models(task, present=False)


class SynchronizedFeedbackTest(unittest.TestCase):
    def task(self):
        offset = np.eye(4)
        offset[2, 3] = 0.12
        return SimpleNamespace(
            wrist_to_tcp=offset, payload_center=None, payload_pose=None, model_stamp=None,
            world_links=(), link_generation=0,
            now=Mock(return_value=10.0), physics={"contact_timeout": 0.5},
            latest_joints=JointState(name=list(JOINT_NAMES), position=[2.0] * 6),
        )

    def test_rigid_grasp_stays_fixed_during_translation_and_rotation(self):
        task = self.task()
        wrist = Pose()
        wrist.orientation.w = 1.0
        payload = Pose(position=Point(x=0.01, y=-0.02, z=0.17))
        message = LinkStates(name=[f"{OBJECT_ID}::{OBJECT_ID}", "arm_sim::link_6"],
                             pose=[payload, wrist])
        GazeboShelfTask.on_links(task, message)
        reference = GazeboShelfTask.local_payload_center(task)
        np.testing.assert_allclose(reference, [0.01, -0.02, 0.05], atol=1e-12)

        # The tool moves and rotates 90 degrees about Y; the object stays fixed
        # inside it. Unrelated joint feedback must not create apparent slip.
        wrist.position.x, wrist.position.y, wrist.position.z = 1.0, 2.0, 3.0
        wrist.orientation.y = wrist.orientation.w = 2**-0.5
        payload.position.x, payload.position.y, payload.position.z = 1.17, 1.98, 2.99
        task.now.return_value = 10.1
        GazeboShelfTask.on_links(task, message)
        task.latest_joints.position = [-2.0] * 6
        task.now.return_value = 10.2
        np.testing.assert_allclose(GazeboShelfTask.local_payload_center(task), reference, atol=1e-12)

        # Actual motion inside the fingers still registers as slip.
        payload.position.x += 0.03
        GazeboShelfTask.on_links(task, message)
        self.assertAlmostEqual(np.linalg.norm(GazeboShelfTask.local_payload_center(task) - reference), 0.03)

    def test_missing_wrist_does_not_refresh_or_mix_payload_samples(self):
        task = self.task()
        task.payload_center = np.array([0.0, 0.0, 0.05])
        task.payload_pose = Pose()
        task.model_stamp = 10.0
        task.now.return_value = 10.6
        GazeboShelfTask.on_links(task, LinkStates(
            name=[f"{OBJECT_ID}::{OBJECT_ID}"], pose=[Pose()]))
        self.assertEqual(task.model_stamp, 10.0)
        with self.assertRaisesRegex(RuntimeError, "stale synchronized"):
            GazeboShelfTask.local_payload_center(task)


class ContactGateTest(unittest.TestCase):
    def test_motion_result_wait_allows_slow_but_advancing_physics(self):
        task = SimpleNamespace(now=Mock(side_effect=[100.0, 100.1, 100.2, 100.3]))
        future = Mock()
        future.done.side_effect = [False, False, False, True]
        future.result.return_value = "complete"
        with patch("arm_moveit_config.gazebo_task.time.monotonic",
                   side_effect=[0, 60, 60, 120, 120, 180, 180]), patch("arm_moveit_config.gazebo_task.rclpy.spin_once"):
            self.assertEqual(GazeboShelfTask.wait_simulation_result(task, future, 2), "complete")

    def test_motion_result_wait_rejects_a_stopped_simulation_clock(self):
        task = SimpleNamespace(now=Mock(return_value=100.0))
        future = Mock()
        future.done.return_value = False
        with patch("arm_moveit_config.gazebo_task.time.monotonic", side_effect=[0, 31]):
            with self.assertRaisesRegex(TimeoutError, "clock stopped"):
                GazeboShelfTask.wait_simulation_result(task, future, 2)

    def task(self):
        task = SimpleNamespace(
            ownership=Mock(), task_running=True, tool_tilt=0.0,
            now=Mock(return_value=10.0), physics={"min_contact_force": 0.2, "contact_timeout": 0.5,
                                               "max_grasp_drift": 0.018},
            contact_forces={"left": 0.0, "right": 0.0},
            last_contact={"left": -float("inf"), "right": -float("inf")},
        )
        return task

    def test_only_sufficient_payload_contact_passes_both_fingers(self):
        task = self.task()
        for side in ("left", "right"):
            msg = ContactsState(states=[ContactState(
                collision1_name=f"arm_sim::finger_{side}::collision",
                collision2_name=f"{OBJECT_ID}::{OBJECT_ID}::collision",
                total_wrench=Wrench(force=Vector3(x=3.0)),
            )])
            GazeboShelfTask.on_contact(task, side, msg)
        self.assertTrue(GazeboShelfTask.bilateral_contact(task))
        task.now.return_value += 0.6
        self.assertFalse(GazeboShelfTask.bilateral_contact(task))

    def test_touching_shelf_or_one_finger_does_not_count_as_grasp(self):
        task = self.task()
        msg = ContactsState(states=[ContactState(
            collision1_name="arm_sim::finger_left::collision",
            collision2_name="shelf_environment::cabinet_shelf::collision",
            total_wrench=Wrench(force=Vector3(x=5.0)),
        )])
        GazeboShelfTask.on_contact(task, "left", msg)
        self.assertFalse(GazeboShelfTask.bilateral_contact(task))
        task.last_contact["left"] = 10.0
        self.assertFalse(GazeboShelfTask.bilateral_contact(task))

    def test_contact_loss_or_slip_cancels_motion(self):
        for contact, drift, age in ((False, 0.0, 0.0), (True, 0.03, 0.0), (True, 0.0, 0.6)):
            with self.subTest(contact=contact, drift=drift, age=age):
                task = self.task()
                task.sync_enabled = task.holding = True
                task.fault = None
                task.payload_pose = Pose()
                task.model_stamp = 10.0 - age
                task.grasp_center = np.zeros(3)
                task.local_payload_center = Mock(return_value=np.array([drift, 0, 0]))
                task.bilateral_contact = Mock(return_value=contact)
                task.active_goal = Mock()
                task.hold_arm = Mock()
                task.report = Mock()
                task.stage = "lift"
                GazeboShelfTask.monitor(task)
                self.assertIsNotNone(task.fault)
                task.active_goal.cancel_goal_async.assert_called_once()
                task.hold_arm.assert_called_once()


class SceneHandoffTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "scene.lock"
        self.old = SceneOwnership(path)
        self.new = SceneOwnership(path)
        self.addCleanup(self.old.close)
        self.addCleanup(self.new.close)
        self.assertTrue(self.old.claim())
        self.old.release()
        self.future = Future()
        scene = ShelfScene()
        self.task = SimpleNamespace(
            ownership=self.old, task_running=False, sync_enabled=True, sync_future=None,
            holding=False, fault=None, payload_pose=Pose(), stage="success", contact_forces={},
            measured_payload=Mock(return_value=scene.payload(scene.place_center)),
            sync_client=Mock(call_async=Mock(return_value=self.future)), report=Mock(),
        )
        self.task.on_sync_done = lambda future: GazeboShelfTask.on_sync_done(self.task, future)

    def test_new_task_waits_for_old_update_and_old_writer_cannot_resume(self):
        GazeboShelfTask.monitor(self.task)
        self.assertFalse(self.new.claim(), "An in-flight update must keep ownership")
        self.future.set_result(SimpleNamespace(success=True))
        self.assertTrue(self.new.claim())
        # The new task holds ownership while using virtual payload positions.
        GazeboShelfTask.monitor(self.task)
        self.task.sync_client.call_async.assert_called_once()
        self.new.release()
        GazeboShelfTask.monitor(self.task)
        self.assertTrue(self.old.superseded)
        self.task.sync_client.call_async.assert_called_once()
        self.task.report.publish.assert_called_once()

    def test_active_task_keeps_ownership_after_update_acknowledgement(self):
        self.assertTrue(self.old.claim())
        self.task.task_running = True
        GazeboShelfTask.monitor(self.task)
        self.future.set_result(SimpleNamespace(success=True))
        self.assertFalse(self.new.claim())

    def test_idle_drains_pending_update_before_releasing_ownership(self):
        self.assertTrue(self.old.claim())
        self.task.task_running = True
        GazeboShelfTask.monitor(self.task)

        def acknowledge(node, future):
            self.assertFalse(self.new.claim())
            self.assertFalse(node.sync_enabled)
            future.set_result(SimpleNamespace(success=True))

        self.task.stop_sync = lambda: GazeboShelfTask.stop_sync(self.task)
        with patch("arm_moveit_config.gazebo_task.rclpy.spin_until_future_complete", side_effect=acknowledge):
            GazeboShelfTask.idle(self.task)
        self.assertTrue(self.new.claim())
        self.assertIsNone(self.task.sync_future)

    def test_deferred_done_callback_does_not_clear_a_new_update(self):
        old_future = Future()
        old_future.set_result(SimpleNamespace(success=True))
        self.task.sync_future = old_future
        with patch("arm_moveit_config.gazebo_task.rclpy.spin_until_future_complete"):
            GazeboShelfTask.stop_sync(self.task)
        self.task.sync_enabled = True
        GazeboShelfTask.monitor(self.task)
        self.task.on_sync_done(old_future)
        self.assertIs(self.task.sync_future, self.future)
        self.assertFalse(self.new.claim())
        self.future.set_result(SimpleNamespace(success=True))
        self.assertTrue(self.new.claim())

    def test_failed_update_is_acknowledged_before_idle_owner_hands_over(self):
        GazeboShelfTask.monitor(self.task)
        self.assertFalse(self.new.claim())
        self.future.set_result(SimpleNamespace(success=False))
        self.assertIn("rejected", self.task.fault)
        self.assertTrue(self.new.claim())

    def test_unacknowledged_update_does_not_release_the_lock(self):
        self.assertTrue(self.old.claim())
        self.task.task_running = True
        GazeboShelfTask.monitor(self.task)
        self.task.stop_sync = lambda: GazeboShelfTask.stop_sync(self.task)
        with patch("arm_moveit_config.gazebo_task.rclpy.spin_until_future_complete"):
            with self.assertRaisesRegex(RuntimeError, "acknowledged"):
                GazeboShelfTask.idle(self.task)
        self.assertFalse(self.new.claim())

    def test_contention_times_out_without_claiming_the_scene(self):
        self.assertTrue(self.old.claim())
        task = SimpleNamespace(ownership=self.new)
        with self.assertRaisesRegex(RuntimeError, "Another shelf task"):
            GazeboShelfTask.acquire_scene(task, timeout=0)
        self.assertTrue(self.old.resume())
