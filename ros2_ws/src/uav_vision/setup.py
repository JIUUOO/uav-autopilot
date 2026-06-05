from setuptools import find_packages, setup

package_name = 'uav_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'candidate_manager = uav_vision.candidate_manager_node:main',
            'gemini_scout_report = uav_vision.gemini_scout_report_node:main',
            'frame_quality_selector = uav_vision.frame_quality_selector_node:main',
            'gemini_frame_analyzer = uav_vision.gemini_frame_analyzer_node:main',
        ],
    },
)
