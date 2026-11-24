"""Gazebo physics, effort controllers, robot state and optional RViz."""

from arm_description.launch import robot_arguments, robot_description
from arm_description.resources import package_file
from arm_control.poses import load_named_poses
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joint_state_spawner, arm_spawner, gripper_spawner = [
        Node(
            package="controller_manager", executable="spawner", output="screen",
            arguments=[controller, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
        )
        for controller in ("joint_state_broadcaster", "arm_controller", "gripper_controller")
    ]
    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )
    spawn = Node(
        package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-entity", "arm_sim", "-topic", "robot_description", "-timeout", "60"],
    )
    auto_home = Node(
        package="arm_control", executable="send_trajectory_goal.py", output="screen",
        arguments=["--pose", LaunchConfiguration("initial_pose"), "--duration", "4",
                   "--server-timeout", "30", "--result-timeout", "30"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("auto_home")),
    )

    def after_success(*actions):
        def callback(event, context):
            if event.returncode != 0:
                return [Shutdown(reason="Gazebo robot/controller startup failed.")]
            return list(actions)
        return callback

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true", description="Enable RViz."),
        DeclareLaunchArgument(
            "gazebo_verbose", default_value="false", description="Verbose Gazebo logging.",
        ),
        DeclareLaunchArgument("world", default_value=str(package_file("arm_bringup", "worlds/arm.world"))),
        DeclareLaunchArgument("auto_home", default_value="true"),
        DeclareLaunchArgument("initial_pose", default_value="home", choices=sorted(load_named_poses())),
        *robot_arguments(),
        Node(
            package="robot_state_publisher", executable="robot_state_publisher", output="screen",
            parameters=[robot_description(), {
                "use_sim_time": True, "publish_frequency": 100.0,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "gui": "false",
                "verbose": LaunchConfiguration("gazebo_verbose"),
                "world": LaunchConfiguration("world"),
                "server_required": "true",
            }.items(),
        ),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_success(joint_state_spawner))),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_spawner, on_exit=after_success(arm_spawner, gripper_spawner),
        )),
        RegisterEventHandler(OnProcessExit(target_action=arm_spawner, on_exit=after_success(auto_home))),
        RegisterEventHandler(OnProcessExit(target_action=gripper_spawner, on_exit=after_success())),
        spawn,
        Node(
            package="rviz2", executable="rviz2", output="screen",
            arguments=["-d", str(package_file("arm_bringup", "rviz/arm.rviz"))],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
