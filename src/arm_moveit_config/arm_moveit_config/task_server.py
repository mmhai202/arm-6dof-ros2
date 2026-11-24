"""One cancellable pick-and-place Action, independent of the RViz client."""

import signal
import threading
import time
import uuid

from arm_control.trajectory import MotionStopError
from arm_description.resources import load_yaml
from arm_task_interfaces.action import PickPlace
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .gazebo_task import GazeboShelfTask
from .perception_input import PERCEPTION_PARAMETERS
from .shelf_scene import ShelfScene
from .task_state import TaskCanceled


ACTION_NAME = "/shelf_pick_place"
MOTION_FIELDS = (
    "velocity_scale", "acceleration_scale", "approach", "lift", "withdraw_clearance",
    "transfer_z", "place_clearance",
)


def goal_motion(goal):
    config = load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml")
    overrides = {} if goal.use_default_motion else {
        key: getattr(goal, key) for key in MOTION_FIELDS
    }
    config["motion"].update(overrides)
    ShelfScene(config)  # Reject invalid settings before ownership or motion.
    return overrides


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("shelf_pick_place_server")
        self.stopping = threading.Event()
        self.gate = threading.Lock()
        self.busy = False
        self.blocked = False
        defaults = load_yaml("arm_moveit_config", "config/shelf_pick_place.yaml")["perception"]
        self.perception_options = {}
        for parameter, key in PERCEPTION_PARAMETERS.items():
            self.declare_parameter(parameter, defaults[key])
            self.perception_options[key] = self.get_parameter(parameter).value
        self.server = ActionServer(
            self, PickPlace, ACTION_NAME, self.execute,
            callback_group=ReentrantCallbackGroup(), goal_callback=self.accept_goal,
            cancel_callback=lambda handle: CancelResponse.ACCEPT,
        )
        self.get_logger().info(f"Pick-and-place Action ready: {ACTION_NAME}")

    def accept_goal(self, request):
        try:
            goal_motion(request)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f"Rejected task settings: {error}")
            return GoalResponse.REJECT
        with self.gate:
            if self.busy or self.blocked or self.stopping.is_set():
                return GoalResponse.REJECT
            # Reserve before accepting; simultaneous clients cannot both start.
            self.busy = True
        return GoalResponse.ACCEPT

    def execute(self, handle):
        started = time.monotonic()
        task = None
        owned = False
        result = PickPlace.Result(success=True, error_code=PickPlace.Result.SUCCESS,
                                  message="Pick-and-place completed.")

        def feedback(progress):
            handle.publish_feedback(PickPlace.Feedback(
                phase=progress.phase, stage=progress.stage,
                completed_stages=progress.completed, total_stages=progress.total,
                holding=task.holding, elapsed_seconds=time.monotonic() - started,
            ))

        try:
            # The server's executor handles goals/cancel requests. Only this
            # callback spins the worker node, so its existing ROS waits remain
            # single-threaded and never re-enter the Action server executor.
            task = GazeboShelfTask(motion_overrides=goal_motion(handle.request),
                                   perception_overrides=self.perception_options,
                                   use_global_arguments=False,
                                   node_name=f"shelf_task_{uuid.uuid4().hex[:8]}")
            task.cancel_requested = lambda: handle.is_cancel_requested or self.stopping.is_set()
            task.progress_callback = feedback
            task.report_progress("initializing", "acquire_scene")
            task.acquire_scene()
            owned = True
            task.run("execute")
            task.check_cancel()
            if task.fault:
                raise RuntimeError(task.fault)
        except TaskCanceled as error:
            result.success = False
            result.error_code = PickPlace.Result.CANCELED
            result.message = str(error)
        except Exception as error:
            if isinstance(error, MotionStopError):
                self.blocked = True
            result.success = False
            result.error_code = PickPlace.Result.FAILED
            result.message = str(error)
            self.get_logger().error(f"Pick-and-place failed: {error}")
        finally:
            try:
                if task is not None:
                    result.final_stage = task.stage
                    # Cleanup must finish even after cancellation. Keep the
                    # admission gate closed until writes have been drained.
                    task.cancel_requested = lambda: False
                    task.report_progress("cleanup", result.final_stage)
                    if owned:
                        if not result.success:
                            task.retain_after_failure(result.message)
                        task.stop_sync()
            except Exception as error:
                self.blocked = True
                result.success = False
                result.error_code = PickPlace.Result.FAILED
                result.message += f" Cleanup failed: {error}"
            finally:
                try:
                    if task is not None:
                        task.progress_callback = lambda progress: None
                        try:
                            task.destroy_node()
                        finally:
                            task.ownership.close()
                except Exception as error:
                    self.blocked = True
                    result.success = False
                    result.error_code = PickPlace.Result.FAILED
                    result.message += f" Worker shutdown failed: {error}"
                finally:
                    result.elapsed_seconds = time.monotonic() - started
                    with self.gate:
                        self.busy = False

        if self.blocked:
            result.message += " Server blocked; inspect controllers and scene before restarting it."

        if handle.is_cancel_requested and result.error_code != PickPlace.Result.FAILED:
            result.success = False
            result.error_code = PickPlace.Result.CANCELED
            result.message = "Task canceled; active motion stopped and scene cleanup completed."
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = PickPlaceServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def stop(signum, frame):
        # Do not tear down ROS or interrupt cleanup on a repeated signal.
        node.stopping.set()
        executor.wake()

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        while rclpy.ok() and not node.stopping.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stopping.set()
        # Keep serving result requests while the worker confirms cancellation.
        while node.busy and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
        for _ in range(5):
            executor.spin_once(timeout_sec=0.05)
        executor.shutdown(timeout_sec=2)
        node.server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
