# PX4 Quadcopter Controller — MATLAB Replication

## Project goal

Replicate the PX4 multicopter control pipeline in MATLAB, from mission
auto-mode down to actuator outputs, plus the sensor stack and EKF2
estimator that PX4 normally feeds the controller from. The simulation
targets the **Holybro Pixhawk 6X (V6X_6 hardware revision)** sensor
suite paired with a **u-blox NEO-M9N** GNSS module.

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

## Current status

- [x] Plant model (QuadrotorDynamics.m)
- [x] Rate controller
- [x] Attitude controller
- [x] Position controller
- [x] Mission/navigator
- [x] End-to-end mission sim
- [ ] Sensor models (IMU x3, baro x2, mag x2, GNSS)
- [ ] Sensor voter / selection
- [ ] EKF2 24-state subset (predict + baro + GNSS + mag + gravity)
- [ ] Output predictor
- [ ] Estimator-feed toggle in run_interactive

## Mission Plan
- Create a mission for multicopter in a 3d world like around 500 meter squared mission and visualize it using matlab tools.
