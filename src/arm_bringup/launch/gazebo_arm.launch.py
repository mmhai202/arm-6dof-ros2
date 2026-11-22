"""Optional Gazebo Classic backend using the same robot and controllers."""

from arm_description.launch import robot_arguments, robot_description
from arm_description.resources import package_file
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spawners = [
        Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[controller, "--controller-manager", "/controller_manager"],
        )
        for controller in ("joint_state_broadcaster", "arm_controller")
    ]
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true", description="Enable RViz."),
        DeclareLaunchArgument(
            "use_gazebo_gui", default_value="true", description="Enable Gazebo GUI.",
        ),
        DeclareLaunchArgument(
            "gazebo_verbose", default_value="false", description="Verbose Gazebo logging.",
        ),
        *robot_arguments(
            ros2_control_plugin="gazebo_ros2_control/GazeboSystem",
            ros2_control_system_name="ArmGazeboSystem",
        ),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", output="screen",
            parameters=[robot_description(gazebo=True), {"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "gui": LaunchConfiguration("use_gazebo_gui"),
                "verbose": LaunchConfiguration("gazebo_verbose"),
            }.items(),
        ),
        Node(
            package="gazebo_ros", executable="spawn_entity.py", output="screen",
            arguments=["-entity", "arm_sim", "-topic", "robot_description"],
        ),
        *spawners,
        Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", str(package_file("arm_bringup", "rviz/arm.rviz"))],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
