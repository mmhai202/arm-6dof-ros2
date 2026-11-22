"""Fake-system controllers, robot state, optional auto-home, and RViz."""

from arm_control.poses import load_named_poses
from arm_description.launch import robot_arguments, robot_description
from arm_description.resources import package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joint_state_spawner, arm_spawner = [
        Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[controller, "--controller-manager", "/controller_manager"],
        )
        for controller in ("joint_state_broadcaster", "arm_controller")
    ]
    auto_home = Node(
        package="arm_control", executable="send_trajectory_goal.py", output="screen",
        arguments=[
            "--pose", LaunchConfiguration("initial_pose"), "--duration", "2",
            "--server-timeout", "10", "--result-timeout", "10",
        ],
        condition=IfCondition(LaunchConfiguration("auto_home")),
    )
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen",
        arguments=["-d", str(package_file("arm_bringup", "rviz/arm.rviz"))],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true", description="Enable RViz."),
        DeclareLaunchArgument(
            "auto_home", default_value="true", description="Move home after controller activation.",
        ),
        DeclareLaunchArgument(
            "initial_pose", default_value="home", choices=sorted(load_named_poses()),
            description="Named pose used for the initial motion when auto_home is enabled.",
        ),
        *robot_arguments(),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[robot_description()], output="screen",
        ),
        Node(
            package="controller_manager", executable="ros2_control_node", output="screen",
            parameters=[str(package_file("arm_control", "config/controllers.yaml"))],
            remappings=[("~/robot_description", "/robot_description")],
        ),
        joint_state_spawner,
        arm_spawner,
        RegisterEventHandler(OnProcessExit(
            target_action=arm_spawner, on_exit=[TimerAction(period=1.0, actions=[auto_home])],
        )),
        TimerAction(period=2.0, actions=[rviz]),
    ])
