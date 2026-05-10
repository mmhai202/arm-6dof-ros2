from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    ros2_control_system_name = LaunchConfiguration("ros2_control_system_name")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare("arm_control"), "config", "controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arm_bringup"), "rviz", "arm.rviz"]
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

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Enable RViz in the baseline launch.",
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
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            rviz_node,
        ]
    )
