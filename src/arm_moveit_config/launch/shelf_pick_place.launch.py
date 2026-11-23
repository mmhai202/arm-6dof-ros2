"""Cabinet-to-box task with contact-only Gazebo grasping."""

from arm_description.resources import package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
        DeclareLaunchArgument("use_gazebo_gui", default_value="false"),
        DeclareLaunchArgument("task_mode", default_value="execute", choices=["scene", "plan", "execute"]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_file(
            "arm_moveit_config", "launch/moveit.launch.py"))), launch_arguments={
                "use_moveit_rviz": LaunchConfiguration("use_moveit_rviz"),
                "use_gazebo_gui": LaunchConfiguration("use_gazebo_gui"),
                "rviz_config": str(package_file("arm_moveit_config", "rviz/pick_place.rviz")),
                # Task plans its own home motion through MoveIt; avoid competing goals.
                "auto_home": "false",
            }.items()),
        TimerAction(period=4.0, actions=[Node(
            package="arm_moveit_config", executable="shelf_pick_place.py", output="screen",
            parameters=[{"mode": LaunchConfiguration("task_mode"),
                         "use_sim_time": True}],
        )]),
    ])
