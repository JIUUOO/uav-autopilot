from glob import glob
from setuptools import find_packages, setup

package_name = 'uav_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm = uav_bringup.arm_node:main',
            'disarm = uav_bringup.disarm_node:main',
            'battery_monitor = uav_bringup.battery_monitor_node:main',
            'guided_takeoff_loiter = uav_bringup.guided_takeoff_loiter_node:main',
            'guided_takeoff_loiter_land = uav_bringup.guided_takeoff_loiter_land_node:main',
            'bounded_scout_mission = uav_bringup.bounded_scout_mission_node:main',
            'front_rectangle_search_mission = uav_bringup.front_rectangle_search_mission_node:main',
            'topdown_target_localization_mission = uav_bringup.topdown_target_localization_mission_node:main',
        ],
    },
)
