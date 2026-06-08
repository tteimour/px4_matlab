"""Bridge node: OpenVINS odometry observer.

VIO runs as observer only — does NOT feed into PX4 EKF2.
PX4 continues navigating with GPS. The bridge publishes:
  - /ov_msckf/position_ned: OV position converted to NED for comparison
  - /quadcopter_markers: Foxglove 3D visualization (VIO vs ground truth)
  - /gps/fix: NavSatFix from PX4 global position

OpenVINS Z-up to NED conversion: NED = [OV.x, -OV.y, -OV.z]
"""

import math
from collections import deque

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def _quat_rotate(qw: float, qx: float, qy: float, qz: float,
                 vx: float, vy: float, vz: float) -> tuple[float, float, float]:
    """Rotate vector (vx,vy,vz) by quaternion (qw,qx,qy,qz)."""
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _build_drone_markers(
    stamp,
    frame_id: str,
    px: float, py: float, pz: float,
    qw: float, qx: float, qy: float, qz: float,
    color: tuple[float, float, float],
    ns: str,
    id_offset: int,
) -> list:
    """Build quadcopter markers (body + arms + propellers + heading arrow)."""
    markers = []
    arm_len = 0.25
    arm_angles = [math.radians(a) for a in (45, 135, 225, 315)]

    # Motor positions: body-frame offsets rotated to world
    motor_world = []
    for angle in arm_angles:
        bx = arm_len * math.cos(angle)
        by = arm_len * math.sin(angle)
        wx, wy, wz = _quat_rotate(qw, qx, qy, qz, bx, by, 0.0)
        motor_world.append((px + wx, py + wy, pz + wz))

    mid = id_offset

    # --- Body (flat cube) ---
    body = Marker()
    body.header.stamp = stamp
    body.header.frame_id = frame_id
    body.ns = ns
    body.id = mid; mid += 1
    body.type = Marker.CUBE
    body.action = Marker.ADD
    body.pose.position.x = px
    body.pose.position.y = py
    body.pose.position.z = pz
    body.pose.orientation.w = qw
    body.pose.orientation.x = qx
    body.pose.orientation.y = qy
    body.pose.orientation.z = qz
    body.scale.x = 0.12
    body.scale.y = 0.12
    body.scale.z = 0.03
    body.color.r, body.color.g, body.color.b = color
    body.color.a = 1.0
    markers.append(body)

    # --- Arms (two diagonal lines through center) ---
    arms = Marker()
    arms.header.stamp = stamp
    arms.header.frame_id = frame_id
    arms.ns = ns
    arms.id = mid; mid += 1
    arms.type = Marker.LINE_LIST
    arms.action = Marker.ADD
    arms.scale.x = 0.02
    arms.pose.orientation.w = 1.0
    arms.color.r, arms.color.g, arms.color.b = color
    arms.color.a = 1.0
    for i, j in [(0, 2), (1, 3)]:
        arms.points.append(Point(
            x=motor_world[i][0], y=motor_world[i][1], z=motor_world[i][2]))
        arms.points.append(Point(
            x=motor_world[j][0], y=motor_world[j][1], z=motor_world[j][2]))
    markers.append(arms)

    for i, (mx, my, mz) in enumerate(motor_world):
        prop = Marker()
        prop.header.stamp = stamp
        prop.header.frame_id = frame_id
        prop.ns = ns
        prop.id = mid; mid += 1
        prop.type = Marker.CYLINDER
        prop.action = Marker.ADD
        prop.pose.position.x = mx
        prop.pose.position.y = my
        prop.pose.position.z = mz
        prop.pose.orientation.w = qw
        prop.pose.orientation.x = qx
        prop.pose.orientation.y = qy
        prop.pose.orientation.z = qz
        prop.scale.x = 0.12
        prop.scale.y = 0.12
        prop.scale.z = 0.005
        prop.color.r, prop.color.g, prop.color.b = color
        prop.color.a = 0.7
        markers.append(prop)

    # --- Heading arrow ---
    arrow = Marker()
    arrow.header.stamp = stamp
    arrow.header.frame_id = frame_id
    arrow.ns = ns
    arrow.id = mid; mid += 1
    arrow.type = Marker.ARROW
    arrow.action = Marker.ADD
    arrow.pose.position.x = px
    arrow.pose.position.y = py
    arrow.pose.position.z = pz
    arrow.pose.orientation.w = qw
    arrow.pose.orientation.x = qx
    arrow.pose.orientation.y = qy
    arrow.pose.orientation.z = qz
    arrow.scale.x = 0.3
    arrow.scale.y = 0.03
    arrow.scale.z = 0.03
    arrow.color.r, arrow.color.g, arrow.color.b = color
    arrow.color.a = 1.0
    markers.append(arrow)

    return markers


class OpenVINSPX4Bridge(Node):
    def __init__(self):
        super().__init__("openvins_px4_bridge")

        self.odom_sub = self.create_subscription(
            Odometry,
            "/ov_msckf/odomimu",
            self._odom_callback,
            10,
        )

        # VIO runs as observer only — do NOT feed into PX4 EKF2.
        # PX4 continues navigating with GPS.

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.px4_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._px4_callback,
            px4_qos,
        )

        self.ov_ned_pub = self.create_publisher(
            PointStamped,
            "/ov_msckf/position_ned",
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, "/quadcopter_markers", 10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.gps_sub = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self._gps_callback,
            px4_qos,
        )
        self.navsat_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)

        self._last_px4_ned = [0.0, 0.0, 0.0]
        self._last_px4_heading = 0.0
        self._msg_count = 0
        self._marker_skip = 0

        # Trajectory trails (keep last 5 min at ~20 Hz = 6000 points)
        self._vio_trail: deque[Point] = deque(maxlen=6000)
        self._gt_trail: deque[Point] = deque(maxlen=6000)

        self.get_logger().info("OpenVINS-PX4 bridge started")

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # Publish OV position converted to NED for Foxglove comparison
        ned_pt = PointStamped()
        ned_pt.header = msg.header
        ned_pt.point.x = p.x
        ned_pt.point.y = -p.y
        ned_pt.point.z = -p.z
        self.ov_ned_pub.publish(ned_pt)

        self._msg_count += 1
        if self._msg_count % 100 == 1:
            self.get_logger().info(
                f"VIO #{self._msg_count}: "
                f"NED=({ned_pt.point.x:.2f}, {ned_pt.point.y:.2f}, {ned_pt.point.z:.2f}) "
                f"PX4=({self._last_px4_ned[0]:.2f}, "
                f"{self._last_px4_ned[1]:.2f}, {self._last_px4_ned[2]:.2f})"
            )

        # Publish quadcopter markers at ~20 Hz (OV odom is ~125 Hz)
        self._marker_skip += 1
        if self._marker_skip % 6 == 0:
            ma = MarkerArray()
            stamp = msg.header.stamp
            frame = msg.header.frame_id

            # Red drone — VIO estimate (OV native Z-up frame)
            ma.markers.extend(_build_drone_markers(
                stamp, frame,
                p.x, p.y, p.z,
                q.w, q.x, q.y, q.z,
                color=(0.8, 0.0, 0.0), ns="vio", id_offset=0,
            ))

            # Green drone — PX4 ground truth (NED → Z-up: x, -y, -z)
            gt = self._last_px4_ned
            h = self._last_px4_heading
            gt_x, gt_y, gt_z = gt[0], -gt[1], -gt[2]
            # NED heading (CW from north) → Z-up yaw: negate angle
            gt_qw = math.cos(-h / 2.0)
            gt_qz = math.sin(-h / 2.0)
            ma.markers.extend(_build_drone_markers(
                stamp, frame,
                gt_x, gt_y, gt_z,
                gt_qw, 0.0, 0.0, gt_qz,
                color=(0.0, 0.8, 0.0), ns="groundtruth", id_offset=100,
            ))

            # TFs so Foxglove 3D panel can follow either drone
            tf_vio = TransformStamped()
            tf_vio.header.stamp = stamp
            tf_vio.header.frame_id = frame
            tf_vio.child_frame_id = "drone_vio"
            tf_vio.transform.translation.x = p.x
            tf_vio.transform.translation.y = p.y
            tf_vio.transform.translation.z = p.z
            tf_vio.transform.rotation.w = q.w
            tf_vio.transform.rotation.x = q.x
            tf_vio.transform.rotation.y = q.y
            tf_vio.transform.rotation.z = q.z

            tf_gt = TransformStamped()
            tf_gt.header.stamp = stamp
            tf_gt.header.frame_id = frame
            tf_gt.child_frame_id = "drone_gt"
            tf_gt.transform.translation.x = gt_x
            tf_gt.transform.translation.y = gt_y
            tf_gt.transform.translation.z = gt_z
            tf_gt.transform.rotation.w = gt_qw
            tf_gt.transform.rotation.z = gt_qz

            self.tf_broadcaster.sendTransform([tf_vio, tf_gt])

            # Trajectory trails
            self._vio_trail.append(Point(x=p.x, y=p.y, z=p.z))
            self._gt_trail.append(Point(x=gt_x, y=gt_y, z=gt_z))

            # VIO trail (red)
            vio_trail = Marker()
            vio_trail.header.stamp = stamp
            vio_trail.header.frame_id = frame
            vio_trail.ns = "trail"
            vio_trail.id = 200
            vio_trail.type = Marker.LINE_STRIP
            vio_trail.action = Marker.ADD
            vio_trail.scale.x = 0.02
            vio_trail.pose.orientation.w = 1.0
            vio_trail.color.r = 0.8
            vio_trail.color.a = 0.8
            vio_trail.points = list(self._vio_trail)
            ma.markers.append(vio_trail)

            # Ground truth trail (green)
            gt_trail = Marker()
            gt_trail.header.stamp = stamp
            gt_trail.header.frame_id = frame
            gt_trail.ns = "trail"
            gt_trail.id = 201
            gt_trail.type = Marker.LINE_STRIP
            gt_trail.action = Marker.ADD
            gt_trail.scale.x = 0.02
            gt_trail.pose.orientation.w = 1.0
            gt_trail.color.g = 0.8
            gt_trail.color.a = 0.8
            gt_trail.points = list(self._gt_trail)
            ma.markers.append(gt_trail)

            self.marker_pub.publish(ma)

    def _px4_callback(self, msg: VehicleLocalPosition):
        if msg.xy_valid and msg.z_valid:
            self._last_px4_ned = [msg.x, msg.y, msg.z]
            if msg.heading_good_for_control:
                self._last_px4_heading = msg.heading

    def _gps_callback(self, msg: VehicleGlobalPosition):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "gps"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = msg.lat
        fix.longitude = msg.lon
        fix.altitude = float(msg.alt)
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        fix.position_covariance = [
            float(msg.eph ** 2), 0.0, 0.0,
            0.0, float(msg.eph ** 2), 0.0,
            0.0, 0.0, float(msg.epv ** 2),
        ]
        self.navsat_pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = OpenVINSPX4Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
