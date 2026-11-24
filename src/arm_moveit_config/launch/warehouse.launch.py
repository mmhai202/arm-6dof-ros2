"""Factory stock rearrangement and camera-guided order fulfillment."""

from arm_description.resources import package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    options = {"use_sim_time": True, "use_perception": True,
               "randomize_payload": LaunchConfiguration("randomize_payload"),
               "perception_seed": LaunchConfiguration("perception_seed")}
    return LaunchDescription([
        DeclareLaunchArgument("use_moveit_rviz", default_value="true"),
        DeclareLaunchArgument("task_mode", default_value="scene", choices=["scene", "plan", "execute"]),
        DeclareLaunchArgument("randomize_payload", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("perception_seed", default_value="-1"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_file(
            "arm_moveit_config", "launch/moveit.launch.py"))), launch_arguments={
                "use_moveit_rviz": LaunchConfiguration("use_moveit_rviz"),
                "rviz_config": str(package_file("arm_moveit_config", "rviz/warehouse.rviz")),
                "auto_home": "false",
            }.items()),
        Node(package="arm_perception", executable="warehouse_detector.py", output="screen",
             parameters=[{"use_sim_time": True, "camera_config": str(package_file(
                 "arm_perception", "config/warehouse_camera.yaml"))}]),
        Node(package="arm_moveit_config", executable="warehouse_display.py", output="screen",
             parameters=[{"use_sim_time": True}]),
        TimerAction(period=4.0, actions=[
            Node(package="arm_moveit_config", executable="warehouse_task.py", output="screen",
                 parameters=[options, {"mode": LaunchConfiguration("task_mode")}]),
            Node(package="arm_moveit_config", executable="warehouse_server.py", output="screen",
                 parameters=[options]),
        ]),
    ])
