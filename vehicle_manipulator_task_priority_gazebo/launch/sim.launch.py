"""Gazebo Harmonic with the pick-and-place world, the mobile uArm, robot_state_publisher and the ROS <-> Gazebo bridge.

    ros2 launch vehicle_manipulator_task_priority_gazebo sim.launch.py              # with the Gazebo window
    ros2 launch vehicle_manipulator_task_priority_gazebo sim.launch.py gui:=false   # headless (server only)

The suction joint exists when the robot is spawned (DetachableJoint starts attached); it is released once, a few
seconds after start-up, before anything moves.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("vehicle_manipulator_task_priority_gazebo"))
    gz_launch = Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"
    world = str(share / "worlds" / "pick_place.sdf")
    robot_description = ParameterValue(Command(["xacro ", str(share / "urdf" / "robot.urdf.xacro")]), value_type=str)
    x, y, yaw = LaunchConfiguration("x"), LaunchConfiguration("y"), LaunchConfiguration("yaw")

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="open the Gazebo window"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        # package:// mesh URIs and model://aruco_box are resolved through these paths.
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share.parent)),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share / "models")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_launch)),
                                 launch_arguments={"gz_args": f"-r {world}"}.items(),
                                 condition=IfCondition(LaunchConfiguration("gui"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_launch)),
                                 launch_arguments={"gz_args": f"-r -s --headless-rendering {world}"}.items(),
                                 condition=UnlessCondition(LaunchConfiguration("gui"))),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "use_sim_time": True}]),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-name", "mobile_swiftpro", "-topic", "robot_description",
                        "-x", x, "-y", y, "-z", "0.01", "-Y", yaw]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="screen",
             parameters=[{"config_file": str(share / "config" / "bridge.yaml"), "use_sim_time": True}]),
        TimerAction(period=6.0, actions=[ExecuteProcess(
            cmd=["gz", "topic", "-t", "/suction/detach", "-m", "gz.msgs.Empty", "-p", " "], output="screen")]),
    ])
