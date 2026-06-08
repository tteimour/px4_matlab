from setuptools import setup

package_name = "openvins_px4_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/openvins_sitl.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Teymur",
    maintainer_email="teymur@todo.todo",
    description="Bridge between OpenVINS odometry and PX4 VehicleOdometry",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge_node = openvins_px4_bridge.bridge_node:main",
        ],
    },
)
