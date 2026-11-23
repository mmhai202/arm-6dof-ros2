"""One Gazebo robot description shared by MoveIt and robot state publishing."""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


ROBOT_ARGUMENTS = {
    "visual_mode": ("cad", "Robot visual mode: cad or primitive."),
    "collision_mode": ("simple", "Robot collision geometry mode."),
}


def robot_arguments(**defaults):
    return [
        DeclareLaunchArgument(
            name, default_value=defaults.get(name, default), description=description,
        )
        for name, (default, description) in ROBOT_ARGUMENTS.items()
    ]


def robot_description():
    command = [
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "arm.urdf.xacro"]),
    ]
    mappings = {name: LaunchConfiguration(name) for name in ROBOT_ARGUMENTS}
    mappings["gazebo_controller_config"] = PathJoinSubstitution(
        [FindPackageShare("arm_control"), "config", "controllers.yaml"]
    )
    for name, value in mappings.items():
        command.extend([" ", f"{name}:=", value])
    return {"robot_description": ParameterValue(Command(command), value_type=str)}
