from setuptools import setup

package_name = "openvins_matlab_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/openvins_matlab_unity.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Teymur",
    maintainer_email="teymur@todo.todo",
    description="Relay the Unity NavCamera stream into OpenVINS (no PX4 dependency).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "unity_image_bridge = openvins_matlab_bridge.unity_image_bridge:main",
        ],
    },
)
