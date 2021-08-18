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
from launch.actions.execute_process import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    data_root_path = "/home/atas/data/"
    specific_bag = "lidar_no_tf_rosbag2_2021_08_17-14_56_00/"
    bag = data_root_path + specific_bag

    share_dir = get_package_share_directory('sensor_drivers_bringup')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(share_dir, 'config',
                                   'rviz.rviz'),
        description='Full path to the RVIZ config file to use')

    static_tf_lidar_data = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '50',
                   'lidar_sensor_link', 'lidar_data_link']
    )
    static_tf_lidar_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '50',
                   'lidar_sensor_link', 'lidar_imu_link']
    )
    static_tf_xsens_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1000',
                   'imu_link', 'xsens_imu_link']
    )

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r' '0.5', bag],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        # change it to screen if you wanna see RVIZ output in terminal
        # output={'both': 'log'},
        output='screen'
    )

    return LaunchDescription([
        static_tf_lidar_data,
        static_tf_lidar_imu,
        static_tf_xsens_imu,
        bag_play,
        declare_rviz_config_file_cmd,
        #rviz_node
    ])
