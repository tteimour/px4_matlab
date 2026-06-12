#!/bin/bash
# Launch OpenVINS + PX4 bridge for SITL
#
# Prerequisites (run in separate terminals BEFORE this script):
#   Terminal 1: MicroXRCEAgent udp4 -p 8888
#   Terminal 2: cd /home/teymur/git/SynapLine/synap-px4 && make px4_sitl gazebo-classic_iris_vio
#
# After OpenVINS initializes (needs drone flying ~5m with attitude changes):
#   PX4 shell: param set EKF2_EV_CTRL 15
#   PX4 shell: param set EKF2_HGT_REF 3

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
    echo "Usage: $0 [--help]"
    echo ""
    echo "Launch OpenVINS VIO + PX4 bridge for Gazebo SITL."
    echo ""
    echo "Prerequisites (run BEFORE this script):"
    echo "  1. MicroXRCEAgent udp4 -p 8888"
    echo "  2. cd /home/teymur/git/SynapLine/synap-px4 && make px4_sitl gazebo-classic_iris_vio"
    echo ""
    echo "After VIO initializes, enable fusion in PX4 shell:"
    echo "  param set EKF2_EV_CTRL 15"
    echo "  param set EKF2_HGT_REF 3"
    exit 0
fi

# Check if microXRCE-DDS agent is running
if ! pgrep -x MicroXRCEAgent > /dev/null 2>&1; then
    echo "WARNING: MicroXRCEAgent not detected. Start it first:"
    echo "  MicroXRCEAgent udp4 -p 8888"
    echo ""
fi

# Keep DDS on loopback (all pipeline participants run on this machine)
export ROS_LOCALHOST_ONLY=1

# Source workspaces in correct order
source /opt/ros/humble/setup.bash
source "$REPO_DIR/open_vins/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"

echo "Launching OpenVINS + PX4 bridge..."
echo "  Config: $REPO_DIR/config/px4_sitl/estimator_config.yaml"
echo ""

ros2 launch openvins_px4_bridge openvins_sitl.launch.py \
    config_path:="$REPO_DIR/config/px4_sitl/estimator_config.yaml"
