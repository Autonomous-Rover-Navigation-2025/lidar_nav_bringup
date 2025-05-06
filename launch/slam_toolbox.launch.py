from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_nav_bringup')

    return LaunchDescription([

        # Static TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
            name='static_tf_pub',
            output='screen'
        ),

        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[os.path.join(pkg_dir, 'params', 'mapper_params_online_async.yaml')],
            output='screen'
        ),
    ])
