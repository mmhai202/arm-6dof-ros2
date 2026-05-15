import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    auto_home = LaunchConfiguration("auto_home")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    ros2_control_system_name = LaunchConfiguration("ros2_control_system_name")
    visual_mode = LaunchConfiguration("visual_mode")
    collision_mode = LaunchConfiguration("collision_mode")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("arm_control"), "config", "controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_bringup"), "rviz", "arm.rviz"]
    )
    trajectory_goal_script = os.path.join(
        get_package_prefix("arm_control"),
        "lib",
        "arm_control",
        "send_trajectory_goal.py",
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

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    auto_home_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="python3"),
            trajectory_goal_script,
            "--pose",
            "home",
            "--duration",
            "2",
            "--server-timeout",
            "10",
            "--result-timeout",
            "10",
        ],
        condition=IfCondition(auto_home),
        output="screen",
    )

    auto_home_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[TimerAction(period=1.0, actions=[auto_home_process])],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
        output="screen",
    )
    delayed_rviz_node = TimerAction(period=2.0, actions=[rviz_node])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Enable RViz in the baseline launch.",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="true",
                description="Send the ready-pose home goal once after the baseline controllers activate.",
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
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            auto_home_after_arm_controller,
            delayed_rviz_node,
        ]
    )
