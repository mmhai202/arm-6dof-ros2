from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    gazebo_verbose = LaunchConfiguration("gazebo_verbose")
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
    gazebo_launch_file = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " ",
                "use_gazebo:=true",
                " ",
                "gazebo_controller_config:=",
                controllers_file,
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
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "gui": use_gazebo_gui,
            "verbose": gazebo_verbose,
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "arm_sim", "-topic", "robot_description"],
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
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Enable RViz in the Gazebo launch.",
            ),
            DeclareLaunchArgument(
                "use_gazebo_gui",
                default_value="true",
                description="Enable the Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "gazebo_verbose",
                default_value="false",
                description="Enable verbose Gazebo logging.",
            ),
            DeclareLaunchArgument(
                "ros2_control_plugin",
                default_value="gazebo_ros2_control/GazeboSystem",
                description="ros2_control backend plugin for Gazebo.",
            ),
            DeclareLaunchArgument(
                "ros2_control_system_name",
                default_value="ArmGazeboSystem",
                description="System name used in the ros2_control block while running Gazebo.",
            ),
            robot_state_publisher_node,
            gazebo,
            spawn_entity,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            rviz_node,
        ]
    )
