# PX4 Quadcopter Controller — MATLAB Replication

## Project goal

Replicate the PX4 multicopter control pipeline in MATLAB, from mission
auto-mode down to actuator outputs. EKF and sensor fusion are explicitly
out of scope — the controller consumes ground-truth state from the
plant model directly.

## Scope

In-scope modules (PX4 → MATLAB mapping):
- `navigator` (mission auto, waypoint sequencing)
- `mc_pos_control` (position + velocity loops, NED frame)
- `mc_att_control` (attitude loop, quaternion-based)
- `mc_rate_control` (body-rate loop, PID + feedforward)
- `control_allocator` / mixer (motor mixing for quad-X)

Out of scope:
- EKF2 / LPE / any state estimation
- Sensor drivers, uORB messaging
- Any other type of platform (VTOL, Fixed Wing or)
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

When implementing a MATLAB equivalent, always cite the PX4 source file
and line range you are translating from in a header comment.

## State interface (ground-truth substitution)

Instead of EKF output, the controller receives a struct from the plant: state.position_ned    % [x; y; z] in meters, NED
state.velocity_ned    % [vx; vy; vz] in m/s, NED
state.attitude_q      % [w; x; y; z] quaternion, body-to-NED
state.angular_vel_b   % [p; q; r] rad/s, body frame
state.acceleration_ned

This struct is produced by `QuadrotorDynamics.m` and consumed directly
by the position controller.

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

The sim loop should run at the rate-controller frequency and downsample
the outer loops with counters.

## Coding rules

- No fabricated values. Gains, limits, and constants must be sourced
  from PX4 parameter definitions (`*.yaml` or `params.c` files in the
  reference tree). Cite the parameter name (e.g., MC_PITCHRATE_P).
- If a PX4 behavior is unclear, ask before guessing — do not invent
  logic that "looks right."
- Match PX4 variable names where reasonable (e.g., `_vel_sp`, `_acc_sp`)
  so cross-referencing is easy.
- Each module gets a unit test in `tests/unit/` that verifies behavior
  against a hand-computed case.

## Current status

- [ ] Plant model (QuadrotorDynamics.m)
- [ ] Rate controller
- [ ] Attitude controller
- [ ] Position controller
- [ ] Mission/navigator
- [ ] End-to-end mission sim

## Mission Plan
- Create a mission for multicopter in a 3d world like around 500 meter squared mission and visualize it using matlab tools.
