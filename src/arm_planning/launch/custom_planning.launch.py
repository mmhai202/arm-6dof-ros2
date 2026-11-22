"""Custom planning service on top of the shared fake-system bringup."""

from arm_description.resources import package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true", description="Enable RViz."),
        DeclareLaunchArgument(
            "auto_home", default_value="true",
            description="Auto-home after controller activation.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(package_file("arm_bringup", "launch/view_arm.launch.py"))
            ),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz"),
                "auto_home": LaunchConfiguration("auto_home"),
            }.items(),
        ),
        Node(package="arm_planning", executable="arm_planning_service.py", output="screen"),
    ])
