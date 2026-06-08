#!/usr/bin/env python3
"""Record VIO (OpenVINS) and PX4 local position data for comparison.

Records immediately on start, saves on Ctrl-C (or when tmux session is killed).

Usage:
    source /opt/ros/humble/setup.bash
    source ~/ytu_thesis/openvins_ws/install/setup.bash
    python3 record_vio_comparison.py [--output-dir ~/ytu_thesis/openvins_ws/recorded_data]
"""

import argparse
import csv
import os
import signal
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class VIOComparisonRecorder(Node):
    def __init__(self, output_dir: str):
        super().__init__("vio_comparison_recorder")

        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self.px4_data = []
        self.vio_data = []
        self.px4_count = 0
        self.vio_count = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._px4_callback,
            px4_qos,
        )
        self.create_subscription(
            Odometry,
            "/ov_msckf/odomimu",
            self._vio_callback,
            10,
        )

        self.get_logger().info(f"Recording started. Output: {output_dir}")

    def _px4_callback(self, msg: VehicleLocalPosition):
        if not (msg.xy_valid and msg.z_valid):
            return

        self.px4_data.append({
            "wall_time": time.time(),
            "timestamp_us": msg.timestamp,
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "vx": msg.vx if msg.v_xy_valid else float("nan"),
            "vy": msg.vy if msg.v_xy_valid else float("nan"),
            "vz": msg.vz if msg.v_z_valid else float("nan"),
            "heading": msg.heading,
        })
        self.px4_count += 1
        if self.px4_count % 500 == 0:
            self.get_logger().info(
                f"PX4: {self.px4_count} | pos=({msg.x:.1f}, {msg.y:.1f}, {msg.z:.1f})"
            )

    def _vio_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear

        self.vio_data.append({
            "wall_time": time.time(),
            "ros_stamp_sec": msg.header.stamp.sec,
            "ros_stamp_nsec": msg.header.stamp.nanosec,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "vx": v.x,
            "vy": v.y,
            "vz": v.z,
            "qw": q.w,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
        })
        self.vio_count += 1
        if self.vio_count == 1:
            self.get_logger().info(
                f"VIO first msg! pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )
        if self.vio_count % 500 == 0:
            self.get_logger().info(
                f"VIO: {self.vio_count} | pos=({p.x:.1f}, {p.y:.1f}, {p.z:.1f})"
            )

    def save_data(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")

        if self.px4_data:
            path = os.path.join(self.output_dir, f"px4_{ts}.csv")
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.px4_data[0].keys())
                w.writeheader()
                w.writerows(self.px4_data)
            self.get_logger().info(f"Saved {len(self.px4_data)} PX4 samples -> {path}")

        if self.vio_data:
            path = os.path.join(self.output_dir, f"vio_{ts}.csv")
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.vio_data[0].keys())
                w.writeheader()
                w.writerows(self.vio_data)
            self.get_logger().info(f"Saved {len(self.vio_data)} VIO samples -> {path}")

        if not self.px4_data and not self.vio_data:
            self.get_logger().warn("No data recorded!")


def main():
    parser = argparse.ArgumentParser(description="Record VIO vs PX4 data")
    parser.add_argument(
        "--output-dir",
        default=os.path.expanduser("~/ytu_thesis/openvins_ws/recorded_data"),
    )
    args = parser.parse_args()

    rclpy.init()
    node = VIOComparisonRecorder(args.output_dir)

    # Save on SIGTERM too (tmux kill-session sends this)
    def _shutdown(sig, frame):
        node.save_data()
        node.destroy_node()
        rclpy.shutdown()
        exit(0)

    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_data()
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
