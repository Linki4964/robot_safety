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

import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_safety_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        # ros2 run resolves executables in lib/<pkg>/, so the launchers are
        # installed there explicitly. The console_scripts entry points below are
        # kept as well, which is what makes `python3 -m` and direct `bin/` use
        # work in either install mode.
        (
            os.path.join('lib', package_name),
            glob(os.path.join('scripts', '*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot-safety maintainer',
    maintainer_email='ros2@example.com',
    description=(
        'Runtime state monitor: judges data freshness and physical plausibility '
        'and publishes an aggregated RobotState snapshot.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor = robot_safety_monitor.monitor_node:main',
            'state_reporter = robot_safety_monitor.state_reporter:main',
            'safety_gate = robot_safety_monitor.safety_gate_node:main',
        ],
    },
)
