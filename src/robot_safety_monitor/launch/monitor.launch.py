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

"""Start the runtime state monitor alone, against an already-running robot.

This is the launch entry point for step 1 of the safety-state-machine project.
The monitor is independent of whoever commands the robot: start Gazebo first
(``turtlebot3_world.launch.py``), then this file, and the monitor will report on
whatever the robot is doing.

    ros2 launch robot_safety_monitor monitor.launch.py
    ros2 launch robot_safety_monitor monitor.launch.py autostart_reporter:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('robot_safety_monitor')
    default_params = os.path.join(package_share, 'config', 'monitor_params.yaml')

    params_file = LaunchConfiguration('params_file')
    robot_id = LaunchConfiguration('robot_id')
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    autostart_reporter = LaunchConfiguration('autostart_reporter')

    declared_arguments = [
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with monitor thresholds and topic names.',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='turtlebot3_burger',
            description='Logical robot name written into every snapshot.',
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value='',
            description='Scenario / Gazebo world label written into snapshots.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo /clock as ROS time.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='10.0',
            description='Snapshot publication rate in Hz.',
        ),
        DeclareLaunchArgument(
            'autostart_reporter',
            default_value='false',
            description='Also start the console state reporter.',
        ),
    ]

    monitor = Node(
        package='robot_safety_monitor',
        executable='monitor',
        name='robot_safety_monitor',
        output='screen',
        parameters=[
            params_file,
            {
                'robot_id': robot_id,
                'world_name': world_name,
                'publish_rate_hz': publish_rate_hz,
                # use_sim_time is a per-node setting, deliberately not part of
                # the YAML: it depends on the clock source, not on thresholds.
                'use_sim_time': use_sim_time,
            },
        ],
    )

    reporter = Node(
        package='robot_safety_monitor',
        executable='state_reporter',
        name='robot_safety_state_reporter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(autostart_reporter),
    )

    return LaunchDescription(declared_arguments + [monitor, reporter])
