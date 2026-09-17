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

"""Start monitor and safety gate together -- the observation and enforcement pair.

    ros2 launch robot_safety_monitor safety_gate.launch.py

The gate publishes to ``/cmd_vel_safe`` by default so that enforcement is
explicit rather than implied. For the gate to actually restrain the robot, the
robot must subscribe to that topic instead of ``/cmd_vel``. With the stock
TurtleBot3 Gazebo model that means remapping the diff-drive plugin's subscriber
on the gzserver command line, or republishing. See the package README.
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
    world_name = LaunchConfiguration('world_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    block_level = LaunchConfiguration('block_level')
    autostart_reporter = LaunchConfiguration('autostart_reporter')

    declared_arguments = [
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Monitor thresholds (also read by the gate for topics).',
        ),
        DeclareLaunchArgument('world_name', default_value='',
                              description='Scenario label written into snapshots.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'block_level', default_value='2',
            description='Minimum spec level S1..S4 that makes the gate refuse a '
                        'command. 2 blocks on warnings and above.',
        ),
        DeclareLaunchArgument('autostart_reporter', default_value='true'),
    ]

    monitor = Node(
        package='robot_safety_monitor',
        executable='monitor',
        name='robot_safety_monitor',
        output='screen',
        parameters=[
            params_file,
            {
                'world_name': world_name,
                'use_sim_time': use_sim_time,
                'topic.cmd_vel': '/cmd_vel',
            },
        ],
    )

    gate = Node(
        package='robot_safety_monitor',
        executable='safety_gate',
        name='robot_safety_gate',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'gate.block_level': block_level,
                'topic.actuator_cmd_vel': '/cmd_vel',
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

    return LaunchDescription(declared_arguments + [monitor, gate, reporter])
