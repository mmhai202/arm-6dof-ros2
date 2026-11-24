"""Cabinet-to-box task with contact-only Gazebo grasping."""

from arm_description.resources import package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
        DeclareLaunchArgument("task_mode", default_value="scene", choices=["scene", "plan", "execute"]),
        DeclareLaunchArgument("use_perception", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("randomize_payload", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("perception_seed", default_value="-1"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_file(
            "arm_moveit_config", "launch/moveit.launch.py"))), launch_arguments={
                "use_moveit_rviz": LaunchConfiguration("use_moveit_rviz"),
                "rviz_config": str(package_file("arm_moveit_config", "rviz/pick_place.rviz")),
                # Task plans its own home motion through MoveIt; avoid competing goals.
                "auto_home": "false",
            }.items()),
        Node(package="arm_perception", executable="rgbd_detector.py", output="screen",
             condition=IfCondition(LaunchConfiguration("use_perception")),
             parameters=[{"use_sim_time": True}]),
        TimerAction(period=4.0, actions=[Node(
            package="arm_moveit_config", executable="shelf_pick_place.py", output="screen",
            parameters=[{"mode": LaunchConfiguration("task_mode"),
                         "use_sim_time": True,
                         "use_perception": LaunchConfiguration("use_perception"),
                         "randomize_payload": LaunchConfiguration("randomize_payload"),
                         "perception_seed": LaunchConfiguration("perception_seed")}],
        ), Node(package="arm_moveit_config", executable="pick_place_server.py", output="screen",
                parameters=[{"use_sim_time": True,
                             "use_perception": LaunchConfiguration("use_perception"),
                             "randomize_payload": LaunchConfiguration("randomize_payload"),
                             "perception_seed": LaunchConfiguration("perception_seed")}])]),
    ])
