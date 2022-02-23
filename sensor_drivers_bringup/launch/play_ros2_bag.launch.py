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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():

    data_root_path = "/home/atas/exp/"
    specific_bag = "rosbag2_2022_02_22-15_11_58/"
    bag = data_root_path + specific_bag

    share_dir = get_package_share_directory('sensor_drivers_bringup')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(share_dir, 'config',
                                   'vox_nav_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    
    static_tf_lidar_data = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '-3.14', '50',
                   'lidar_sensor_link', 'lidar_data_link']
    )

    thorvald_bringup_share_dir = get_package_share_directory(
        'thorvald_bringup')
    thorvald_example_robots_share_dir = get_package_share_directory(
        'thorvald_example_robots')
    thorvald_desc_share_dir = get_package_share_directory(
        'thorvald_description')

    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration("use_rviz")
    robot_model_params = LaunchConfiguration('robot_model_params')
    model_extras = LaunchConfiguration('model_extras')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='...')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='...')
    declare_robot_model_params = DeclareLaunchArgument(
        'robot_model_params',
        default_value=os.path.join(
            thorvald_example_robots_share_dir, 'config', 'thorvald_ii_4wd4ws_std', 'thorvald_ii_4wd4ws_std.yaml'),
        description='...')
    declare_model_extras = DeclareLaunchArgument(
        'model_extras',
        default_value=os.path.join(
            thorvald_desc_share_dir, 'urdf', 'sensor_modules', 'real_full_sensor_suite.xacro'),
        description='...')
    
    thorvald_description_share_dir = get_package_share_directory(
        'thorvald_description')
        
    xacro_file_name = 'thorvald_model.xacro'
    xacro_full_dir = os.path.join(
        thorvald_description_share_dir, 'urdf', xacro_file_name)
    
    declare_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command(['xacro ', xacro_full_dir,
                                                   ' robot_name:=', 'thorvald_ii',
                                                   ' robot_config:=', robot_model_params,
                                                   ' tf_prefix:=', '',
                                                   ' model_extras:=', model_extras])}],
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static')])

    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r' ' 1.0', bag],
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
        bag_play,
        declare_rviz_config_file_cmd,
        rviz_node, 
        declare_robot_model_params,
        declare_model_extras,
        declare_use_sim_time,
        declare_robot_state_publisher_node
        
    
    ])
