#!/usr/bin/env bash
# VIO pipeline health check. Run it WHILE a flight is going with VIO enabled
# (so the topics are live). Localises why OpenVINS diverges:
#   - did OpenVINS initialise, and is the recovered gravity tilted?
#   - is the camera actually reaching OpenVINS (/down_cam/image_raw)?
#   - is the IMU flowing, and is odometry being published?
# (no 'set -u' -- ROS 2 setup.bash references unset vars and would abort.)
LOG=/tmp/px4_openvins.log

echo "===== 1) OpenVINS init / gravity recovery (from $LOG) ====="
if [ -f "$LOG" ]; then
  grep -hE "init-|gravity|did not converge|Done initial|started" "$LOG" | tail -25
  echo "  (|g| should be ~9.81 with X,Y small. Large X,Y = tilted gravity = bad"
  echo "   visual features. 'did not converge' / no init line = never initialised.)"
else
  echo "  no $LOG -- did the VIO toggle launch OpenVINS?"
fi

echo
echo "===== 2) nodes + topic rates (sourced ROS 2) ====="
unset LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash 2>/dev/null
echo "-- relevant nodes --"
ros2 node list 2>/dev/null | grep -E "ov_msckf|unity|vio|px4_matlab" || echo "  (none found)"
echo "-- topic rates (5 s sample each) --"
for t in /synapsim/nav /down_cam/image_raw /vio_imu/data /ov_msckf/odomimu; do
  printf "  %-24s " "$t"
  timeout 5 ros2 topic hz "$t" 2>/dev/null | grep -m1 -i average || echo "(silent / not publishing)"
done

echo
echo "===== reading guide ====="
echo "  /synapsim/nav      0  -> Unity not publishing the camera (scene not playing?)"
echo "  /down_cam/image_raw 0 -> image bridge not relaying  -> OpenVINS has NO vision"
echo "  /vio_imu/data      ~200 expected (from MATLAB)"
echo "  /ov_msckf/odomimu  ~125 once initialised; 0 = not initialised / no vision"
