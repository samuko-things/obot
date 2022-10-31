# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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


from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess

import xacro


def generate_launch_description():
    slam_mapping_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("obot_slam_mapping"), 'config', 'slam_mapping_params.yaml')],
        #     remappings=[
        #     # ('/odom', '/turtlesim1/turtle1/pose'),
        #     # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        # ]
        )


    return LaunchDescription([
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'),
        
        slam_mapping_node,
        
    ])