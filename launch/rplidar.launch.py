#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_nav_bringup_dir = get_package_share_directory('lidar_nav_bringup')
    rplidar_config = os.path.join(lidar_nav_bringup_dir, 'params',
                                  'rplidar_params.yaml')

    return LaunchDescription([
        Node(package='rplidar_ros',
             executable='rplidar_node',
             name='rplidar_node',
             parameters=[rplidar_config],
             output='screen'),
    ])
