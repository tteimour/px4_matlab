# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# PX4 Quadcopter Controller — MATLAB Replication

## Project goal

Replicate the PX4 multicopter control pipeline in MATLAB, from mission
auto-mode down to actuator outputs, plus the sensor stack and EKF2
estimator that PX4 normally feeds the controller from. The simulation
targets the **Holybro Pixhawk 6X (V6X_6 hardware revision)** sensor
suite paired with a **u-blox NEO-M9N** GNSS module.

## Running the simulation

All entry points are in `sim/`. They self-add paths via `addpath`, so they
can be invoked from any working directory in MATLAB.

```matlab
% Interactive GCS flight deck (dark theme, tabbed UI, manual sticks)
run_interactive          % sim/run_interactive.m

% Non-interactive 500 m-square mission (saves mission_result.png, tracking_errors.png)
run_mission              % sim/run_mission.m

% Drive the attitude autotune state machine end-to-end against the plant
run_autotune             % sim/run_autotune.m

% Compare OpenVINS VIO against MATLAB EKF2 (requires sim_log in base workspace
% and a recorded ROS 2 bag from /ov_msckf/odomimu)
compare_vio_ekf('vio_run')   % sim/compare_vio_ekf.m
```

`run_interactive` has a **SENSORS tab** (estimator feed ON/OFF toggle). When
ON, the full sensor→voter→EKF2→OutputPredictor chain replaces ground-truth
state. Both modes produce the same controller-facing state struct shape.

## Running tests

From the MATLAB command window:

```matlab
cd tests/unit
run_all_tests           % runs all test_*.m in alphabetical order

% Run a single test
test_ekf2_basics
test_rate_controller
% etc.
```

Each test calls `error()` on failure, so `run_all_tests` catches and counts
failures. No test framework dependency.

## Scope

In-scope modules (PX4 → MATLAB mapping):
- `navigator` (mission auto, waypoint sequencing)
- `mc_pos_control` (position + velocity loops, NED frame)
- `mc_att_control` (attitude loop, quaternion-based)
- `mc_rate_control` (body-rate loop, PID + feedforward)
- `control_allocator` / mixer (motor mixing for quad-X)
- Sensor drivers — chip-accurate models for the V6X_6 set:
  IMUs ICM-45686, IIM-42652, ADIS-16470; baros ICP-201XX (internal),
  BMP388 (external); mags BMM150 (internal), IST8310 (external);
  GNSS u-blox NEO-M9N
- `sensors` voted_sensors_update — multi-IMU / multi-baro / multi-mag
  selection, priority + health + error counters, switchover
- `ekf2` — 24-state EKF subset: predict + covariance, fusions for
  baro, GNSS pos+vel, mag (heading + 3D), gravity. Output predictor
  with delay-compensated buffer
- `mc_autotune_attitude_control` — system-identification attitude/rate
  auto-tuner: ARX recursive-least-squares (`system_identification` lib),
  GMVC PID design (`pid_design` lib), per-axis excitation + state machine

Out of scope:
- LPE / other estimators (only EKF2 is replicated)
- Optical flow, range finder, airspeed, sideslip, drag, external
  vision, terrain fusion (deferred — not needed for current sim)
- uORB messaging plumbing (replaced by direct struct passing)
- Any other type of platform (VTOL, Fixed Wing)
- Failsafe / commander state machine (beyond minimal arming)
- Logging infrastructure

## Reference codebase

PX4 source is at `/home/teymur/git/SynapLine/synap-px4`. Treat it as read-only. Key files
to consult when implementing each module:

- Position control: `src/modules/mc_pos_control/PositionControl/PositionControl.cpp`
- Attitude control: `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`
- Rate control: `src/lib/rate_control/RateControl.cpp`
- Control allocation: `src/modules/control_allocator/`
- Autotune: `src/modules/mc_autotune_attitude_control/`,
  `src/lib/system_identification/` (arx_rls, system_identification,
  signal_generator), `src/lib/pid_design/pid_design.hpp`
- Navigator: `src/modules/navigator/`
- Sensor drivers: `src/drivers/imu/`, `src/drivers/barometer/`,
  `src/drivers/magnetometer/`, `src/drivers/gps/`
- Sensor voter / selection: `src/modules/sensors/voted_sensors_update.cpp`,
  `src/modules/sensors/vehicle_imu/`, `src/modules/sensors/vehicle_air_data/`,
  `src/modules/sensors/vehicle_magnetometer/`, `src/modules/sensors/vehicle_gps_position/`
- EKF2 core: `src/modules/ekf2/EKF/ekf.cpp`, `src/modules/ekf2/EKF/control.cpp`,
  `src/modules/ekf2/EKF/covariance.cpp`, `src/modules/ekf2/EKF/aid_sources/`,
  `src/modules/ekf2/EKF/output_predictor/`
- Board config: `boards/px4/fmu-v6x/default.px4board`,
  `boards/px4/fmu-v6x/init/rc.board_sensors`,
  `boards/px4/fmu-v6x/src/spi.cpp`

When implementing a MATLAB equivalent, always cite the PX4 source file
and line range you are translating from in a header comment.

## State interface

Two state-feed paths are supported, switchable at sim time:

1. **Ground-truth feed (legacy / debug)**. The controller receives a
   struct from the plant directly:
   ```
   state.position_ned    % [x; y; z] in meters, NED
   state.velocity_ned    % [vx; vy; vz] in m/s, NED
   state.attitude_q      % [w; x; y; z] quaternion, body-to-NED
   state.angular_vel_b   % [p; q; r] rad/s, body frame
   state.acceleration_ned
   ```
2. **Estimator feed (PX4-faithful)**. `QuadrotorDynamics` produces
   ground truth → sensor models corrupt it into chip-accurate samples
   at each chip's native rate → `voted_sensors_update` selects the
   primary per type → `EKF2` consumes those and emits the same struct
   shape above. The output predictor produces low-latency state at IMU
   rate.

Both paths produce the same struct shape so the controller is
unchanged. The toggle lives in `run_interactive.m`.

## Sensor / estimator data flow

```
QuadrotorDynamics (ground truth, ~1 kHz)
    │
    ├─→ ImuICM45686 / ImuIIM42652 / ImuADIS16470  (sensor-rate, ~1–2 kHz)
    ├─→ BaroICP201XX / BaroBMP388                  (~50–100 Hz)
    ├─→ MagBMM150 / MagIST8310                     (~100 Hz)
    └─→ GnssM9N                                    (10 Hz)
                                                 │
                                                 ▼
                            voted_sensors_update (priority + health)
                                                 │
                                                 ▼
                                              EKF2 (24-state)
                                                 │
                                                 ▼
                                       OutputPredictor (IMU-rate)
                                                 │
                                                 ▼
                                          state struct → controller
```

## EKF2 state vector (24-state, PX4 ordering)

Match `src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h`:

| Index | Name        | Size | Description                              |
|-------|-------------|------|------------------------------------------|
| 0–3   | quat        | 4    | body→NED quaternion [w; x; y; z]         |
| 4–6   | vel         | 3    | NED velocity [m/s]                        |
| 7–9   | pos         | 3    | NED position [m] from local origin       |
| 10–12 | gyro_bias   | 3    | gyro bias [rad/s]                         |
| 13–15 | accel_bias  | 3    | accel bias [m/s²]                         |
| 16–18 | mag_I       | 3    | Earth magnetic field, NED [Gauss]         |
| 19–21 | mag_B       | 3    | body magnetic bias [Gauss]                |
| 22–23 | wind_vel    | 2    | horizontal wind N/E [m/s]                 |

## MATLAB conventions

- Use classes (`classdef`) for stateful controllers, plain functions for
  pure math (e.g., quaternion ops, frame transforms).
- All angles in radians. All frames explicitly NED or FRD body —
  no ENU, no aerospace-frame ambiguity.
- Quaternions in [w; x; y; z] order, matching PX4 convention (Hamilton
  product, body-to-NED rotation).
- Time step: fixed dt passed in explicitly. No tic/toc inside controllers.
- Vector convention: column vectors throughout.
- DO not use Simulink. Default to .m files.

## Update rates (match PX4)

Find the rates in PX4 database if you cannot use these:
- Position control: 50–100 Hz
- Attitude control: 250 Hz
- Rate control: 1000 Hz (or matched to IMU rate)
- Mission/navigator: 10 Hz
- IMU sensor sampling: ~1–2 kHz (chip-dependent)
- Baro sampling: 50–100 Hz
- Mag sampling: ~100 Hz
- GNSS: 10 Hz (M9N native)
- EKF2 prediction: IMU rate; fusion at sensor delivery rate

The sim loop should run at the rate-controller frequency and downsample
the outer loops with counters. Sensors run from a lightweight time
queue so each chip fires at its own native rate independent of the
sim step.

## Coding rules

- No fabricated values. Gains, limits, and constants must be sourced
  from PX4 parameter definitions (`*.yaml` or `params.c` files in the
  reference tree) or chip driver code / datasheets. Cite the parameter
  name (e.g., MC_PITCHRATE_P, EKF2_GYR_NOISE) or driver file:line.
- Sensor noise / bias / scale numbers must come from either the PX4
  driver source for that chip or the chip datasheet. Cite the source
  in the sensor model header.
- If a PX4 behavior is unclear, ask before guessing — do not invent
  logic that "looks right."
- Match PX4 variable names where reasonable (e.g., `_vel_sp`, `_acc_sp`,
  `_state.quat_nominal`, `_imu_sample_delayed`) so cross-referencing is
  easy.
- Each module gets a unit test in `tests/unit/` that verifies behavior
  against a hand-computed case.

## VIO / OpenVINS integration

The `vio/` directory holds the OpenVINS observer pipeline. See
`vio/README.md` for full details. Summary:

```
vio/
  open_vins/          git submodule (rpng/open_vins, unmodified upstream)
  openvins_ws/src/
    openvins_matlab_bridge/   Unity NavCamera → /down_cam/image_raw + launch
    openvins_px4_bridge/      PX4 SITL bridge (needs px4_msgs)
  config/
    matlab_unity/   estimator_config.yaml, kalibr_imu_chain.yaml, kalibr_imucam_chain.yaml
    px4_sitl/       equivalent configs for SITL
```

**First-time build** (after `git submodule update --init vio/open_vins`):

```bash
cd vio/open_vins
colcon build --packages-select ov_core ov_init ov_msckf ov_eval \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

cd ../openvins_ws
colcon build
```

**Run (MATLAB + Unity path)** — source order: `/opt/ros/humble` → `vio/open_vins/install` → `vio/openvins_ws/install`:

1. `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
2. In MATLAB: `run_interactive` → enable **Stream pose to Cesium/Unity**
3. Unity: Play the Quba scene
4. `ros2 launch openvins_matlab_bridge openvins_matlab_unity.launch.py`

Output: `/ov_msckf/odomimu` (`nav_msgs/Odometry`, ~125 Hz).

**Compare VIO vs EKF2**: record a bag during flight
(`ros2 bag record -o vio_run /ov_msckf/odomimu`), then in MATLAB with
`sim_log` in the base workspace: `compare_vio_ekf('vio_run')`. Uses
SE3 Umeyama alignment (`src/math/umeyama_align.m`) before computing ATE
because OpenVINS' gauge yaw and origin are unobservable.

## Unity/Cesium bridge

`src/bridge/CesiumBridge.m` publishes `geometry_msgs/PoseArray` on
`/world/default/pose/info` as a ROS 2 node. Frame convention: Gazebo
ENU/FLU (converted from NED/FRD by `ned_frd_to_enu_flu()`). ROS quaternion
order is `(x,y,z,w)`; px4_matlab uses `[w;x;y;z]` — the bridge handles
the swap. Only `rosbridge_server` is needed on the Unity side; PX4 SITL
and Gazebo are not required.

## Current status

- [x] Plant model (QuadrotorDynamics.m)
- [x] Rate controller
- [x] Attitude controller
- [x] Position controller
- [x] Mission/navigator
- [x] End-to-end mission sim (500 m-square, `sim/run_mission.m`)
- [x] Sensor models (IMU x3, baro x2, mag x2, GNSS)
- [x] Sensor voter / selection
- [x] EKF2 24-state subset (predict + baro + GNSS + mag + gravity)
- [x] Output predictor
- [x] Estimator-feed toggle in run_interactive
- [x] System-identification autotune (ArxRls + SystemIdentification +
      GMVC pid_design + McAutotuneAttitudeControl state machine);
      demo in sim/run_autotune.m, unit tests vs PX4 reference values
- [x] VIO/OpenVINS integration (observer path, compare_vio_ekf)
- [ ] Estimator validation (unit + bench tests)
