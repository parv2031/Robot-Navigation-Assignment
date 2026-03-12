# -------------------------------------------------------------
# Launch File for Robot Navigation Simulation
# -------------------------------------------------------------
# This launch file starts the full simulation and visualization
# environment for the trajectory tracking system.
#
# It launches the following components:
#
# 1. Gazebo simulator with an empty world
# 2. Controller node responsible for trajectory tracking
# 3. RViz for visualization of paths and robot motion
#
# The controller node receives waypoint parameters from a YAML
# configuration file and generates a trajectory that the robot
# follows using a PID controller.
# -------------------------------------------------------------

import os

# LaunchDescription is the container for all launch actions
from launch import LaunchDescription

# IncludeLaunchDescription allows launching another launch file
# (used here to start the Gazebo simulation)
from launch.actions import IncludeLaunchDescription, TimerAction

# Used to load Python-based launch files
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Used to launch ROS2 nodes
from launch_ros.actions import Node

# Utility to locate installed ROS2 packages
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------------------------------------------------
    # Get paths of required ROS2 packages
    # ---------------------------------------------------------
    # robot_navigation -> current project
    # turtlebot3_gazebo -> simulation package for TurtleBot3
    # ---------------------------------------------------------

    robot_nav_pkg = get_package_share_directory('robot_navigation')
    turtlebot_pkg = get_package_share_directory('turtlebot3_gazebo')

    # ---------------------------------------------------------
    # Define file paths used by the launch system
    # ---------------------------------------------------------

    # RViz configuration file used to automatically load
    # visualization settings (topics, frames, displays, etc.)
    rviz_config = os.path.join(robot_nav_pkg, 'launch', 'navigation.rviz')

    # YAML configuration file containing controller parameters
    # such as waypoint coordinates
    param_file = os.path.join(robot_nav_pkg, 'config', 'controller_params.yaml')

    # ---------------------------------------------------------
    # Launch Gazebo with an empty world
    # ---------------------------------------------------------
    # The empty world provides a clean simulation environment
    # without obstacles so that the robot can follow the
    # generated trajectory.
    # ---------------------------------------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot_pkg,
                'launch',
                'empty_world.launch.py'
            )
        )
    )

    # ---------------------------------------------------------
    # Controller Node
    # ---------------------------------------------------------
    # This node performs the following operations:
    #
    # 1. Reads waypoint parameters from YAML
    # 2. Generates a smoothed path
    # 3. Converts the path into a time-parameterized trajectory
    # 4. Tracks the trajectory using a PID controller
    # 5. Publishes velocity commands to /cmd_vel
    #
    # A TimerAction is used to delay startup slightly so that
    # Gazebo and the robot model are fully initialized before
    # the controller begins executing.
    # ---------------------------------------------------------

    controller = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_navigation',
                executable='controller',
                name='controller_node',
                output='screen',
                parameters=[param_file]
            )
        ]
    )

    # ---------------------------------------------------------
    # RViz Visualization
    # ---------------------------------------------------------
    # RViz is launched with a predefined configuration file
    # which automatically loads:
    #
    #  - /planned_path  (smoothed planned trajectory)
    #  - /robot_path    (actual robot trajectory)
    #
    # This allows easy comparison between the desired and
    # executed paths.
    # ---------------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    # ---------------------------------------------------------
    # Return the launch description containing all actions
    # ---------------------------------------------------------

    return LaunchDescription([
        gazebo,
        controller,
        rviz
    ])
