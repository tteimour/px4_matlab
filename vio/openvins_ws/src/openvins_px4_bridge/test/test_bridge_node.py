"""Tests for the OpenVINS-PX4 bridge frame conversion logic."""

import math

import pytest


@pytest.mark.unit
class TestENUToNEDConversion:
    """Test ENU -> NED frame conversion used by the bridge node."""

    def test_position_enu_to_ned(self):
        """ENU position (1, 2, 3) should map to NED (2, 1, -3)."""
        from openvins_px4_bridge.bridge_node import enu_position_to_ned

        ned = enu_position_to_ned(1.0, 2.0, 3.0)
        assert ned == pytest.approx([2.0, 1.0, -3.0])

    def test_position_zero(self):
        from openvins_px4_bridge.bridge_node import enu_position_to_ned

        ned = enu_position_to_ned(0.0, 0.0, 0.0)
        assert ned == pytest.approx([0.0, 0.0, 0.0])

    def test_velocity_enu_to_ned(self):
        """ENU velocity follows same mapping as position."""
        from openvins_px4_bridge.bridge_node import enu_velocity_to_ned

        ned = enu_velocity_to_ned(1.0, 2.0, 3.0)
        assert ned == pytest.approx([2.0, 1.0, -3.0])

    def test_quaternion_enu_to_ned(self):
        """Identity quaternion in ENU (no rotation) should produce a valid NED quaternion.

        ENU identity means body axes = ENU axes.
        In NED, body forward=North=ENU.y, body right=East=ENU.x, body down=-ENU.z.
        The expected NED quaternion for ENU-identity orientation is the
        rotation from FRD body to NED: 90-degree yaw offset.
        """
        from openvins_px4_bridge.bridge_node import enu_quaternion_to_ned_frd

        q_ned = enu_quaternion_to_ned_frd(1.0, 0.0, 0.0, 0.0)
        norm = math.sqrt(sum(c * c for c in q_ned))
        assert norm == pytest.approx(1.0, abs=1e-6)

    def test_quaternion_preserves_unit_norm(self):
        """Converted quaternion must remain unit-length."""
        from openvins_px4_bridge.bridge_node import enu_quaternion_to_ned_frd

        angle = math.pi / 4
        q_ned = enu_quaternion_to_ned_frd(
            math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)
        )
        norm = math.sqrt(sum(c * c for c in q_ned))
        assert norm == pytest.approx(1.0, abs=1e-6)


@pytest.mark.unit
class TestTimestampConversion:
    """Test ROS2 header timestamp to PX4 microseconds conversion."""

    def test_timestamp_conversion(self):
        from openvins_px4_bridge.bridge_node import ros_stamp_to_px4_us

        us = ros_stamp_to_px4_us(10, 500_000_000)
        assert us == 10_500_000

    def test_timestamp_zero(self):
        from openvins_px4_bridge.bridge_node import ros_stamp_to_px4_us

        us = ros_stamp_to_px4_us(0, 0)
        assert us == 0

    def test_timestamp_nanosecond_precision(self):
        from openvins_px4_bridge.bridge_node import ros_stamp_to_px4_us

        us = ros_stamp_to_px4_us(1, 1000)
        assert us == 1_000_001
