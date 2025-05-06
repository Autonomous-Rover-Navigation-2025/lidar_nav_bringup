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
            'launch/lidar_nav_bringup.launch.py', 'launch/rplidar.launch.py',
            'launch/slam_toolbox.launch.py', 'launch/amcl.launch.py'
        ]),
        ('share/' + package_name + '/params', [
            'params/rplidar_params.yaml',
            'params/mapper_params_online_async.yaml',
            'params/amcl_params.yaml',
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
        'console_scripts': [],
    },
)
