"""Launch OpenVINS (observer) on the MATLAB-sim + Unity/Cesium pipeline.

Pipeline (each in its own terminal, started BEFORE this launch):
  1. ros2 launch rosbridge_server rosbridge_websocket_launch.xml   (for Unity)
  2. MATLAB: run_interactive  -> enable the "Stream pose to Cesium/Unity" toggle
            (publishes /world/default/pose/info AND /vio_imu/data)
  3. Unity: Play the Quba scene  (publishes /synapsim/nav)

This launch then:
  - starts unity_image_bridge (/synapsim/nav CompressedImage -> /down_cam/image_raw Image)
  - starts ov_msckf run_subscribe_msckf with the matlab_unity config (2 s later)

No PX4, no Gazebo, no px4_msgs.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = (
    "/home/teymur/git/px4_matlab/vio/config/matlab_unity/estimator_config.yaml"
)

launch_args = [
    DeclareLaunchArgument(
        name="config_path",
        default_value=DEFAULT_CONFIG,
        description="Absolute path to estimator_config.yaml",
    ),
    DeclareLaunchArgument(name="verbosity", default_value="INFO"),
    DeclareLaunchArgument(name="in_topic", default_value="/synapsim/nav"),
    DeclareLaunchArgument(name="out_topic", default_value="/down_cam/image_raw"),
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not os.path.isfile(config_path):
        return [
            LogInfo(msg=f"ERROR: config not found: '{config_path}' — not starting OpenVINS")
        ]

    image_bridge = Node(
        package="openvins_matlab_bridge",
        executable="unity_image_bridge",
        name="unity_image_bridge",
        output="screen",
        parameters=[
            {"in_topic": LaunchConfiguration("in_topic")},
            {"out_topic": LaunchConfiguration("out_topic")},
            {"grayscale": True},
        ],
    )

    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        output="screen",
        parameters=[
            {"verbosity": LaunchConfiguration("verbosity")},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"config_path": config_path},
        ],
    )

    # Image bridge first so /down_cam/image_raw exists; OpenVINS 2 s later.
    return [image_bridge, TimerAction(period=2.0, actions=[openvins_node])]


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
