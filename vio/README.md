# VIO (OpenVINS) integration

OpenVINS visual-inertial odometry running as an **observer** on the
`px4_matlab` + Unity/Cesium pipeline, plus the ROS 2 bridges that feed it.
Consolidated here from `~/ytu_thesis/simulation/` so the estimator, the
bridge workspace, and the configs live in one place on the `thesis` branch.

## Layout

```
vio/
  open_vins/        git submodule -> rpng/open_vins, pinned to 6948812
                    (= upstream master tip at vendor time). Unmodified upstream.
  openvins_ws/      ROS 2 workspace (your code; build/ install/ log/ not tracked)
    src/
      openvins_matlab_bridge/   Unity NavCamera -> /down_cam/image_raw relay
                                + matlab_unity launch. No PX4 dependency.
      openvins_px4_bridge/      PX4 SITL bridge (needs px4_msgs, see below)
    scripts/        analyze_errors.py, plot_vio_comparison.py, record_vio_comparison.py
    launch_openvins.sh          PX4 SITL launcher
  config/           Project estimator configs (kept outside the submodule so
                    they can be tracked here; relative_config_* resolve within
                    each dir, so each is self-contained):
    matlab_unity/   estimator_config.yaml (+ kalibr_imu_chain, kalibr_imucam_chain)
    px4_sitl/       estimator_config.yaml (+ kalibr_*)
```

The OpenVINS source itself is a submodule (upstream, unmodified). Only the
project-specific bits — the bridge packages, scripts, and configs — are tracked
directly in this repo.

## First-time setup

```bash
# from the px4_matlab repo root, after cloning with --recurse-submodules
# (or: git submodule update --init vio/open_vins)

# 1. build OpenVINS
cd vio/open_vins
colcon build --packages-select ov_core ov_init ov_msckf ov_eval \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 2. build the bridge workspace
cd ../openvins_ws
colcon build
```

`px4_msgs` is an `exec_depend` of `openvins_px4_bridge` only. It was **empty** in
the source tree and is not vendored. If you use the PX4 SITL path, clone
`PX4/px4_msgs` (branch matching your PX4 version — unverified which one you used)
into `vio/openvins_ws/src/` before building. The MATLAB + Unity path
(`openvins_matlab_bridge`) does **not** need it.

## Run — MATLAB + Unity (observer) path

Source order matters: `/opt/ros/humble` -> `vio/open_vins/install` ->
`vio/openvins_ws/install`.

Start these first, each in its own terminal:
1. `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`  (for Unity)
2. MATLAB: `run_interactive` -> enable the **Stream pose to Cesium/Unity** toggle
   (publishes `/world/default/pose/info` and `/vio_imu/data`)
3. Unity: Play the Quba scene (publishes `/synapsim/nav`)

Then:
```bash
ros2 launch openvins_matlab_bridge openvins_matlab_unity.launch.py
```
This starts `unity_image_bridge` (`/synapsim/nav` -> `/down_cam/image_raw`) and,
2 s later, `ov_msckf/run_subscribe_msckf` with `config/matlab_unity/`. The launch
default `config_path` points at `vio/config/matlab_unity/estimator_config.yaml`
(absolute path, like the original; override with `config_path:=...`).

Output: `/ov_msckf/odomimu` (`nav_msgs/Odometry`, ~125 Hz) — position in a
gravity-aligned `global` frame (arbitrary yaw + origin from dynamic init),
velocity in the IMU body frame. Track video: `/ov_msckf/trackhist` (publishes
only with a subscriber). View with `ros2 run rqt_image_view rqt_image_view`.

## Run — PX4 SITL path

`openvins_ws/launch_openvins.sh` sources the workspaces and launches
`openvins_px4_bridge` with `config/px4_sitl/estimator_config.yaml`. See the
prerequisites in that script's header (MicroXRCEAgent + PX4 SITL).

## Compare VIO vs MATLAB EKF2

During a flight: `ros2 bag record -o vio_run /ov_msckf/odomimu`. Then in MATLAB
(`sim_log` in base workspace, estimator feed ON): `compare_vio_ekf('vio_run')`.
It SE3-aligns VIO -> NED via `src/math/umeyama_align.m` (gauge yaw/origin are
unobservable, so alignment is required — same as ov_eval ATE) and plots
position N/E/D + ATE, velocity, and speed.

## Provenance

Vendored from `~/ytu_thesis/simulation/` on 2026-06-08. Build/install/log trees
and `recorded_data/` were intentionally excluded (regenerable; colcon bakes
absolute paths so they don't survive relocation). `open_vins/ov_data` (upstream
datasets) comes with the submodule checkout.
