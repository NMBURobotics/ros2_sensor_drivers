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
from launch.actions import DeclareLaunchArgument
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('sensor_drivers_bringup')

    localization_params = LaunchConfiguration('localization_params')

    decleare_localization_params = DeclareLaunchArgument(
        'localization_params',
        default_value=os.path.join(
            share_dir, 'config', 'localization_params.yaml'),
        description='Path to the localization parameters file.')

    ekf_local_filter_node = Node(package='robot_localization',
                                 executable='ekf_node',
                                 name='ekf_local_filter_node',
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('odometry/filtered', 'odometry/local')])

    ekf_global_filter_node = Node(package='robot_localization',
                                  executable='ekf_node',
                                  name='ekf_global_filter_node',
                                  output='screen',
                                  parameters=[localization_params],
                                  remappings=[('odometry/filtered', 'odometry/global')])

    navsat_transform_node = Node(package='robot_localization',
                                 executable='navsat_transform_node',
                                 name='navsat_transform_node',
                                 # change it to screen if you wanna see RVIZ output in terminal
                                 # output={'both': 'log'},
                                 output='screen',
                                 parameters=[localization_params],
                                 remappings=[('imu', '/heading/imu'),
                                             ('gps/fix', '/fix'),
                                             ('gps/filtered', 'gps/filtered'),
                                             ('odometry/gps', 'odometry/gps'),
                                             ('odometry/filtered', 'odometry/global')])

    return LaunchDescription([
        decleare_localization_params,
        ekf_global_filter_node,
        ekf_local_filter_node,
        navsat_transform_node
    ])
