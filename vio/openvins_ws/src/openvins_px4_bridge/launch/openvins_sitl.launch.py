"""Launch OpenVINS + PX4 bridge for SITL.

Starts bridge node first, then OpenVINS with a 2-second delay
(OpenVINS only publishes odometry when subscribers exist).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_PATH = os.path.join(
    os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    ),
    "open_vins",
    "config",
    "px4_sitl",
    "estimator_config.yaml",
)

launch_args = [
    DeclareLaunchArgument(
        name="config_path",
        default_value=CONFIG_PATH,
        description="Absolute path to estimator_config.yaml",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="OpenVINS verbosity: ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
]


def launch_setup(context):
    config_path = LaunchConfiguration("config_path").perform(context)
    if not os.path.isfile(config_path):
        return [
            LogInfo(
                msg=f"ERROR: config not found: '{config_path}' — not starting OpenVINS"
            )
        ]

    bridge_node = Node(
        package="openvins_px4_bridge",
        executable="bridge_node",
        name="openvins_px4_bridge",
        output="screen",
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

    delayed_openvins = TimerAction(period=2.0, actions=[openvins_node])

    return [bridge_node, delayed_openvins]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
