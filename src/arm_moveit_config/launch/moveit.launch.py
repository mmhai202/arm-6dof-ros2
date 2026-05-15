import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r", encoding="utf-8") as file_handle:
        return yaml.safe_load(file_handle)


def load_text(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)

    with open(absolute_path, "r", encoding="utf-8") as file_handle:
        return file_handle.read()


def generate_launch_description():
    use_moveit_rviz = LaunchConfiguration("use_moveit_rviz")
    auto_home = LaunchConfiguration("auto_home")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    ros2_control_system_name = LaunchConfiguration("ros2_control_system_name")
    visual_mode = LaunchConfiguration("visual_mode")
    collision_mode = LaunchConfiguration("collision_mode")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_moveit_config"), "rviz", "moveit.rviz"]
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " ",
                "ros2_control_plugin:=",
                ros2_control_plugin,
                " ",
                "ros2_control_system_name:=",
                ros2_control_system_name,
                " ",
                "visual_mode:=",
                visual_mode,
                " ",
                "collision_mode:=",
                collision_mode,
            ]
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": load_text("arm_moveit_config", "config/arm.srdf")
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "arm_moveit_config", "config/kinematics.yaml"
        )
    }
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "arm_moveit_config", "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_pipeline_config["ompl"].update(
        load_yaml("arm_moveit_config", "config/ompl_planning.yaml")
    )

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_bringup"), "launch", "view_arm.launch.py"]
            )
        ),
        launch_arguments={
            "use_rviz": "false",
            "auto_home": auto_home,
            "ros2_control_plugin": ros2_control_plugin,
            "ros2_control_system_name": ros2_control_system_name,
            "visual_mode": visual_mode,
            "collision_mode": collision_mode,
        }.items(),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            load_yaml("arm_moveit_config", "config/moveit_controllers.yaml"),
            trajectory_execution,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_moveit_rviz),
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )
    delayed_rviz_node = TimerAction(period=2.0, actions=[rviz_node])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_moveit_rviz",
                default_value="true",
                description="Enable RViz while launching MoveIt.",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="true",
                description="Send the baseline ready-pose goal once after the controller activates.",
            ),
            DeclareLaunchArgument(
                "ros2_control_plugin",
                default_value="mock_components/GenericSystem",
                description="ros2_control backend plugin for the robot description.",
            ),
            DeclareLaunchArgument(
                "ros2_control_system_name",
                default_value="ArmFakeSystem",
                description="System name used in the robot description ros2_control block.",
            ),
            DeclareLaunchArgument(
                "visual_mode",
                default_value="cad",
                description="Robot visual mode passed to xacro (cad or primitive).",
            ),
            DeclareLaunchArgument(
                "collision_mode",
                default_value="simple",
                description="Collision mode passed to xacro.",
            ),
            bringup_launch,
            move_group_node,
            delayed_rviz_node,
        ]
    )
