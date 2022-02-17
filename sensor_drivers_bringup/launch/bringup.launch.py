# Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (GroupAction,
                            IncludeLaunchDescription)
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    default_rviz = os.path.join(get_package_share_directory('sensor_drivers_bringup'),
                                'config', 'rviz.rviz')

    imu_share_dir = get_package_share_directory(
        'bluespace_ai_xsens_mti_driver')
    lidar_share_dir = get_package_share_directory(
        'ros2_ouster')
    # zed_share_dir = get_package_share_directory(
    #    'zed_wrapper')
    nmea_share_dir = get_package_share_directory(
        'nmea_navsat_driver')
    realsense_share_dir = get_package_share_directory(
        'realsense_ros2_camera')

    # Specify the actions
    bringup_sensor_group = GroupAction([

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(imu_share_dir, 'launch',
                         'xsens_mti_node.launch.py'))),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(lidar_share_dir, 'launch',
                         'driver_launch.py'))),


        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #    os.path.join(zed_share_dir, 'launch',
        #                 'zed.launch.py'))),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(realsense_share_dir, 'launch',
                         'ros2_intel_realsense.launch.py'))),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nmea_share_dir, 'launch',
                         'nmea_serial_driver.launch.py'))),
    ])

    # Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        # change it to screen if you wanna see RVIZ output in terminal
        output={'both': 'log'},
    )

    imu_fuse_node = Node(
        package='sensor_drivers_bringup',
        executable='imu_fuse_node',
        name='imu_fuse_node',
        #prefix=['xterm -e gdb -ex run --args'],
        output='screen')

    return LaunchDescription([
        bringup_sensor_group,
        rviz_node,
        imu_fuse_node
    ])
