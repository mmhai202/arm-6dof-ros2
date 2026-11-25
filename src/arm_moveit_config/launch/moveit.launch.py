"""MoveIt planning and RViz on top of the shared ros2_control bringup."""

from arm_control.poses import load_named_poses
from arm_control.trajectory import controller_start_tolerance
from arm_description.launch import ROBOT_ARGUMENTS, robot_arguments, robot_description
from arm_description.resources import load_yaml, package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE = "arm_moveit_config"


def generate_launch_description():
    sim_time = {"use_sim_time": True}
    model_parameters = [
        robot_description(),
        {"robot_description_semantic": package_file(PACKAGE, "config/arm.srdf").read_text(
            encoding="utf-8",
        )},
        {"robot_description_kinematics": load_yaml(PACKAGE, "config/kinematics.yaml")},
        {"robot_description_planning": load_yaml(PACKAGE, "config/joint_limits.yaml")},
    ]
    ompl = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]),
            "start_state_max_bounds_error": 0.1,
            **load_yaml(PACKAGE, "config/ompl_planning.yaml"),
        },
    }
    execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": controller_start_tolerance(),
    }
    scene_monitor = {
        "publish_planning_scene": True,
        # RViz's Scene Robot follows these updates, not only its render timer.
        "publish_planning_scene_hz": 60.0,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
    }
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("use_moveit_rviz")),
        parameters=[*model_parameters, sim_time],
    )
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_verbose", default_value="false"),
        DeclareLaunchArgument("world", default_value=str(package_file("arm_bringup", "worlds/arm.world"))),
        DeclareLaunchArgument(
            "use_moveit_rviz", default_value="true", description="Enable MoveIt RViz.",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value=str(package_file(PACKAGE, "rviz/moveit.rviz")),
            description="RViz display configuration.",
        ),
        DeclareLaunchArgument(
            "auto_home", default_value="true", description="Move to the initial pose on startup.",
        ),
        DeclareLaunchArgument(
            "initial_pose", default_value="home", choices=sorted(load_named_poses()),
            description="Named pose used for the initial motion on startup.",
        ),
        *robot_arguments(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_file("arm_bringup", "launch/gazebo_arm.launch.py"))),
            launch_arguments={
                "use_rviz": "false",
                "gazebo_verbose": LaunchConfiguration("gazebo_verbose"),
                "world": LaunchConfiguration("world"),
                "auto_home": LaunchConfiguration("auto_home"),
                "initial_pose": LaunchConfiguration("initial_pose"),
                **{name: LaunchConfiguration(name) for name in ROBOT_ARGUMENTS},
            }.items(),
        ),
        Node(
            package="moveit_ros_move_group", executable="move_group", output="screen",
            parameters=[
                *model_parameters, ompl, load_yaml(PACKAGE, "config/moveit_controllers.yaml"),
                execution, scene_monitor, sim_time,
            ],
        ),
        TimerAction(period=2.0, actions=[rviz]),
    ])
