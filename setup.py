from setuptools import find_packages, setup

package_name = 'lidar_nav_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/nav2.launch.py',
        ]),
        ('share/' + package_name + '/params', [
            'params/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/maps', [
            'maps/my_map.pgm',
            'maps/my_map.yaml',
            'maps/map_lab.pgm',
            'maps/map_lab.yaml',
            'maps/map_ics.pgm',
            'maps/map_ics.yaml',
        ]),
        ('share/' + package_name + '/map_landmarks', [
            'map_landmarks/landmarks_lab.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ias',
    maintainer_email='ias@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcasters = lidar_nav_bringup.static_tf_broadcasters:main ',
            'location_goal_sender = lidar_nav_bringup.location_goal_sender:main ',
        ],
    },
)
