"""Relay the Unity NavCamera stream into OpenVINS.

Unity (cesium-unity-samples Vehicle.cs) publishes the downward camera as
sensor_msgs/CompressedImage (JPEG) on /synapsim/nav via rosbridge. OpenVINS
expects a RAW sensor_msgs/Image on /down_cam/image_raw. This node decodes the
JPEG and republishes it as a mono8 Image, preserving the header stamp (which
Unity copies from the MATLAB pose = sim time, the same clock the IMU uses).

No cv_bridge dependency — decodes with OpenCV (cv2) + numpy directly.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CompressedImage, Image


class UnityImageBridge(Node):
    def __init__(self):
        super().__init__("unity_image_bridge")

        self.declare_parameter("in_topic", "/synapsim/nav")
        self.declare_parameter("out_topic", "/down_cam/image_raw")
        self.declare_parameter("frame_id", "cam0")
        self.declare_parameter("grayscale", True)

        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.grayscale = bool(self.get_parameter("grayscale").value)

        # Subscribe BEST_EFFORT (sensor-data profile): a best-effort subscription
        # matches a publisher offering EITHER reliable or best-effort, so this
        # receives /synapsim/nav regardless of what QoS rosbridge_server offers.
        # (A reliable subscription would silently receive nothing if rosbridge
        # publishes best-effort.)
        self.sub = self.create_subscription(
            CompressedImage, in_topic, self.cb, qos_profile_sensor_data
        )
        # IMPORTANT: OpenVINS's ACTIVE monocular image subscriber is RELIABLE
        # depth-10 (ov_msckf ROS2Visualizer.cpp:213-214; the SensorDataQoS
        # variant is commented out). A best-effort publisher would NOT match it
        # (zero images -> VIO never inits), so publish RELIABLE.
        self.pub = self.create_publisher(Image, out_topic, QoSProfile(depth=10))

        self.count = 0
        self.get_logger().info(
            f"unity_image_bridge: {in_topic} (CompressedImage/jpeg) -> "
            f"{out_topic} (Image, {'mono8' if self.grayscale else 'bgr8'})"
        )

    def cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        flag = cv2.IMREAD_GRAYSCALE if self.grayscale else cv2.IMREAD_COLOR
        img = cv2.imdecode(buf, flag)
        if img is None:
            self.get_logger().warn("JPEG decode failed; dropping frame")
            return

        out = Image()
        out.header.stamp = msg.header.stamp  # preserve sim-time stamp (camera/IMU shared clock)
        out.header.frame_id = self.frame_id
        out.height = int(img.shape[0])
        out.width = int(img.shape[1])
        out.is_bigendian = 0
        if self.grayscale:
            out.encoding = "mono8"
            out.step = out.width
        else:
            out.encoding = "bgr8"
            out.step = out.width * 3
        out.data = np.ascontiguousarray(img).tobytes()
        self.pub.publish(out)

        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(
                f"relayed {self.count} frames ({out.width}x{out.height} {out.encoding})"
            )


def main():
    rclpy.init()
    node = UnityImageBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
