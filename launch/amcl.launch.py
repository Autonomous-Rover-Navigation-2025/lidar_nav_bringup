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
    amcl_config = os.path.join(lidar_nav_bringup_dir, 'params',
                               'amcl_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_config],
        )
    ])
