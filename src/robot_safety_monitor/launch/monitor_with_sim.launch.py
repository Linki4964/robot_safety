# Copyright 2026 robot-safety maintainer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bring up the Gazebo TurtleBot3 world and the runtime state monitor together.

Convenience entry point for experiments:

    export TURTLEBOT3_MODEL=burger
    ros2 launch robot_safety_monitor monitor_with_sim.launch.py

Keep ``monitor.launch.py`` as the entry point when the robot is started by some
other means (real hardware, a different world, a test harness).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    monitor_share = get_package_share_directory('robot_safety_monitor')

    world_name = LaunchConfiguration('world_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart_reporter = LaunchConfiguration('autostart_reporter')

    declared_arguments = [
        DeclareLaunchArgument(
            'world_name',
            default_value='turtlebot3_world',
            description='TurtleBot3 Gazebo world to load (without .world).',
        ),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart_reporter', default_value='true'),
    ]

    # turtlebot3_gazebo ships one launch file per world; the world name selects
    # which one, so the mapping stays in one place here.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_share, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    monitor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(monitor_share, 'launch', 'monitor.launch.py')
        ),
        launch_arguments={
            'world_name': world_name,
            'use_sim_time': use_sim_time,
            'autostart_reporter': autostart_reporter,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [simulation, monitor])
