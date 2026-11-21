"""Pick and place: the simulation, the task-priority controller, the ArUco detector and the behaviour-tree mission.

    ros2 launch vehicle_manipulator_task_priority pick_place.launch.py                   # Gazebo window
    ros2 launch vehicle_manipulator_task_priority pick_place.launch.py gui:=false        # headless
    ros2 launch vehicle_manipulator_task_priority pick_place.launch.py use_aruco:=false  # fixed box position

The mission starts after the robot has settled and the suction joint has been released (start_delay).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("vehicle_manipulator_task_priority"))
    sim = Path(get_package_share_directory("vehicle_manipulator_task_priority_gazebo")) / "launch" / "sim.launch.py"
    params = str(share / "config" / "controller.yaml")
    common = {"use_sim_time": True}
    mission_params = [params, common, {"tree": str(share / "config" / "pick_place.xml")}]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_aruco", default_value="true"),
        DeclareLaunchArgument("start_delay", default_value="10.0"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim)),
                                 launch_arguments={"gui": LaunchConfiguration("gui")}.items()),
        Node(package="vehicle_manipulator_task_priority", executable="controller_node",
             name="task_priority_controller", output="screen", parameters=[params, common]),
        Node(package="vehicle_manipulator_task_priority", executable="aruco_node", name="aruco_detector",
             output="screen", parameters=[params, common], condition=IfCondition(LaunchConfiguration("use_aruco"))),
        TimerAction(period=LaunchConfiguration("start_delay"), actions=[
            Node(package="vehicle_manipulator_task_priority", executable="mission_node", name="mission",
                 output="screen", parameters=mission_params + [{"use_aruco": LaunchConfiguration("use_aruco")}]),
        ]),
    ])
