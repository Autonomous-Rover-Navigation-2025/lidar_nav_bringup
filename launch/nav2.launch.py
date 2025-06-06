import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    lidar_nav_bringup_dir = get_package_share_directory('lidar_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    param_file = os.path.join(lidar_nav_bringup_dir, 'params',
                              'nav2_params.yaml')

    # Declare arguments
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(lidar_nav_bringup_dir, 'maps',
                                   'my_map.yaml'),
        description='Path to the map YAML file')

    declare_global_frame_arg = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global frame used for localization and planning')

    declare_slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether to run SLAM (True) or AMCL (False)')

    # Use LaunchConfiguration references
    map_yaml = LaunchConfiguration('map')
    global_frame = LaunchConfiguration('global_frame')
    slam = LaunchConfiguration('slam')

    # Rewrite params to use map as global_frame
    param_substitutions = {
        'global_frame': global_frame,
        'yaml_filename': map_yaml}
    configured_params = RewrittenYaml(source_file=param_file,
                                      root_key='',
                                      param_rewrites=param_substitutions,
                                      convert_types=True)

    return LaunchDescription([
        declare_map_arg, declare_global_frame_arg, declare_slam_arg,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                                 launch_arguments={
                                     'map': map_yaml,
                                     'use_sim_time': 'False',
                                     'params_file': configured_params,
                                     'autostart': 'True',
                                     'slam': slam
                                 }.items())
    ])
