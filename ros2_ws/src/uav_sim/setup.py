from setuptools import setup

package_name = "uav_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JIUUOO",
    maintainer_email="nninjiuuoo@gmail.com",
    description="Simulation nodes for UAV autonomous mission system",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_vehicle_state_node = uav_sim.sim_vehicle_state_node:main",
            "sim_lidar_slice_node = uav_sim.sim_lidar_slice_node:main",
            "sim_detection_event_node = uav_sim.sim_detection_event_node:main",
        ],
    },
)
