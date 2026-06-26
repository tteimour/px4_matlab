"""Launch the YOLO detector for the SynapLine tracker-lock workflow.

    ros2 launch synap_detection detector.launch.py
    ros2 launch synap_detection detector.launch.py weights:=yolov8s.pt device:=cpu

Numeric params (conf, max_targets, imgsz) are set here; override them with
`ros2 run synap_detection yolo_detector --ros-args -p conf:=0.4` if needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    str_args = {
        'weights': 'yolov8n.pt',
        'device': 'cuda:0',
        'image_topic': '/synapsim/gimbal',
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in str_args.items()]
    params = {k: LaunchConfiguration(k) for k in str_args}
    # Defaults tuned for distant/small targets on the 512x512 gimbal: run
    # inference at a larger imgsz (finer grid -> better small-object recall,
    # upsamples the 512 input) and a lower confidence gate. For more range,
    # use a bigger model: weights:=yolov8m.pt / yolov8x.pt (GPU handles it).
    params.update({'conf': 0.25, 'iou': 0.45, 'imgsz': 960, 'max_targets': 4})
    return LaunchDescription(decls + [
        Node(
            package='synap_detection',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[params],
        ),
    ])
