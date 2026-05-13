from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    auto_home = LaunchConfiguration("auto_home")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_bringup"), "launch", "view_arm.launch.py"]
            )
        ),
        launch_arguments={
            "use_rviz": use_rviz,
            "auto_home": auto_home,
        }.items(),
    )

    planning_service_node = Node(
        package="arm_planning",
        executable="arm_planning_service.py",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Enable RViz for the custom planning mode.",
            ),
            DeclareLaunchArgument(
                "auto_home",
                default_value="false",
                description="Auto-home once after controller activation.",
            ),
            bringup_launch,
            planning_service_node,
        ]
    )
