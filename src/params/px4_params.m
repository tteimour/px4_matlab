function p = px4_params()
% PX4 parameter set + Iris airframe constants for the MATLAB replication.
%
% All controller gains/limits sourced from the PX4 reference tree at
%   /home/teymur/git/SynapLine/synap-px4
% Plant parameters sourced from the Iris SITL model
%   Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf
% and the Quad-X allocator airframe
%   ROMFS/px4fmu_common/init.d/airframes/4001_quad_x
%
% Conventions (per CLAUDE.md):
%   - World frame: NED (North-East-Down)
%   - Body frame:  FRD (Forward-Right-Down)
%   - Quaternions: [w; x; y; z], Hamilton, body-to-NED
%   - All angles in radians; column vectors throughout.

p.g = 9.80665;  % m/s^2

% =====================================================================
% Loop rates (Hz). Source: PX4 module defaults / scheduler.
%   Position control:   50 Hz   (mc_pos_control: MPC_NUM_INTERVAL ~ 20 ms)
%   Attitude control:  250 Hz
%   Rate control:     1000 Hz
%   Mission/navigator:  10 Hz
% =====================================================================
p.rate_hz.rate     = 1000;
p.rate_hz.attitude = 250;
p.rate_hz.position = 50;
p.rate_hz.navigator = 10;

% =====================================================================
% Rate controller
% Source: src/lib/rate_control/rate_control.cpp:71-118
%         src/modules/mc_rate_control/mc_rate_control_params.c
% PX4 applies an outer K multiplier: effective_P = MC_*RATE_P * MC_*RATE_K
% (parallel form). With MC_*RATE_K = 1.0 the effective gains equal the
% nominal P/I/D values.
% =====================================================================
p.rate.gain_p  = [0.15;  0.15;  0.20];   % MC_ROLLRATE_P, MC_PITCHRATE_P, MC_YAWRATE_P
p.rate.gain_i  = [0.20;  0.20;  0.10];   % MC_ROLLRATE_I, MC_PITCHRATE_I, MC_YAWRATE_I
p.rate.gain_d  = [0.003; 0.003; 0.0];    % MC_ROLLRATE_D, MC_PITCHRATE_D, MC_YAWRATE_D
p.rate.gain_ff = [0.0;  0.0;   0.0];    % MC_ROLLRATE_FF, ...
p.rate.gain_k  = [1.0;   1.0;   1.0];    % MC_*RATE_K (global multiplier)
p.rate.int_lim = [0.30; 0.30;  0.30];   % MC_RR_INT_LIM, MC_PR_INT_LIM, MC_YR_INT_LIM
% i-factor cutoff: rate_control.cpp:107 uses radians(400 deg/s)
p.rate.i_factor_cutoff_rate = deg2rad(400);

% =====================================================================
% Attitude controller
% Source: src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:55-114
%         src/modules/mc_att_control/mc_att_control_params.c
% =====================================================================
p.att.gain_p   = [6.5; 6.5; 2.8];       % MC_ROLL_P, MC_PITCH_P, MC_YAW_P
p.att.yaw_weight = 0.4;                 % MC_YAW_WEIGHT
% Rate setpoint clamps (rad/s): MC_ROLLRATE_MAX=220deg/s, MC_PITCHRATE_MAX=220, MC_YAWRATE_MAX=200
p.att.rate_max = deg2rad([220; 220; 200]);

% =====================================================================
% Position controller
% Source: src/modules/mc_pos_control/PositionControl/PositionControl.cpp
%         src/modules/mc_pos_control/multicopter_position_control_*_params.c
% =====================================================================
% Outer position-loop P gains (m/s per m). MPC_XY_P, MPC_Z_P.
p.pos.gain_pos_p = [0.95; 0.95; 1.0];

% Velocity-loop PID gains (m/s^2 per m/s for P and D, per m for I).
% MPC_XY_VEL_{P,I,D}_ACC = 1.8, 0.4, 0.2
% MPC_Z_VEL_{P,I,D}_ACC  = 4.0, 2.0, 0.0
p.pos.gain_vel_p = [1.8; 1.8; 4.0];
p.pos.gain_vel_i = [0.4; 0.4; 2.0];
p.pos.gain_vel_d = [0.2; 0.2; 0.0];

% Velocity limits (m/s). MPC_XY_VEL_MAX, MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN.
% NED z-down: vel_max for "up" applies to negative z, "dn" to positive z.
p.pos.vel_xy_max  = 12.0;
p.pos.vel_z_up    = 3.0;
p.pos.vel_z_down  = 1.5;

% Tilt limit (rad). MPC_TILTMAX_AIR = 45 deg.
p.pos.tilt_max = deg2rad(45);

% Thrust limits/hover. MPC_THR_HOVER, MPC_THR_MIN, MPC_THR_MAX.
p.pos.thr_hover = 0.5;
p.pos.thr_min   = 0.12;
p.pos.thr_max   = 1.0;

% =====================================================================
% Control allocator (Quad-X). Source: 4001_quad_x airframe.
% PX4 numbering (0-indexed):
%   Rotor 0: PX=+1, PY=+1   (Front-Right, CW,  km = +0.05)
%   Rotor 1: PX=-1, PY=-1   (Back-Left,   CW,  km = +0.05)
%   Rotor 2: PX=+1, PY=-1   (Front-Left,  CCW, km = -0.05)
%   Rotor 3: PX=-1, PY=+1   (Back-Right,  CCW, km = -0.05)
% The +/-0.05 originates from CA_ROTORn_KM (default 0.05 with sign flip
% on rotors 2 & 3 in the airframe file).
% =====================================================================
p.alloc.rotor_px = [+1; -1; +1; -1];
p.alloc.rotor_py = [+1; -1; -1; +1];
p.alloc.rotor_km = [+0.05; +0.05; -0.05; -0.05];
p.alloc.n_rotors = 4;

% =====================================================================
% Navigator. Source: src/modules/navigator/navigator_params.c
% =====================================================================
p.nav.acc_rad = 10.0;        % NAV_ACC_RAD (m), horizontal acceptance radius
p.nav.alt_acc_rad = 1.0;     % NAV_MC_ALT_RAD (m) typical default

% =====================================================================
% Manual flight modes (Stabilized, Altitude, Position).
% Source:
%   src/modules/flight_mode_manager/tasks/Sticks/Sticks.cpp
%   src/modules/flight_mode_manager/tasks/Manual*/FlightTaskManual*.cpp
%   src/modules/mc_pos_control/multicopter_position_control_params.c
% =====================================================================
p.man.tilt_max     = deg2rad(35);   % MPC_MAN_TILT_MAX (rad)
p.man.yaw_rate_max = deg2rad(150);  % MPC_MAN_Y_MAX  (rad/s)
p.man.vel_xy_max   = 10.0;          % MPC_VEL_MANUAL (m/s)
p.man.deadzone     = 0.05;          % MPC_HOLD_DZ
p.man.expo         = 0.6;           % MPC_*_MAN_EXPO  (single value reused)
p.man.hold_max_xy  = 0.8;           % MPC_HOLD_MAX_XY: lock pos when |v_xy|<this
p.man.hold_max_z   = 0.6;           % MPC_HOLD_MAX_Z

% =====================================================================
% Auto modes (Mission, RTL, Loiter/Hold, Land, Takeoff).
% Source:
%   src/modules/navigator/{rtl_direct,land,takeoff,loiter}.cpp
%   src/modules/mc_pos_control/multicopter_position_control_params.c
% =====================================================================
p.auto.cruise_speed  = 5.0;         % MPC_XY_CRUISE
p.auto.land_speed    = 0.7;         % MPC_LAND_SPEED
p.auto.land_crawl    = 0.3;         % MPC_LAND_CRWL (slow descent near ground)
p.auto.land_alt1     = 5.0;         % MPC_LAND_ALT1: above this, land_speed
p.auto.land_alt3     = 1.0;         % MPC_LAND_ALT3: below this, land_crawl
p.auto.takeoff_speed = 1.5;         % MPC_TKO_SPEED
p.auto.takeoff_alt   = 5.0;         % MIS_TAKEOFF_ALT (m above ground)
p.auto.rtl_alt       = 15.0;        % RTL_ALT (m above ground)
p.auto.land_alt_ground = 0.10;      % altitude below which we treat as landed

% =====================================================================
% Wind disturbance (NON-PX4 addition).
% Steady NED component plus simplified Dryden / Ornstein-Uhlenbeck
% turbulence (first-order low-passed Gaussian noise per axis with
% correlation time tau and steady-state std sigma). Turbulence is off
% by default; values below are the seed for the UI fields.
% =====================================================================
p.wind.steady      = [0; 0; 0];      % m/s, NED
p.wind.turb_enable = false;
p.wind.turb_sigma  = 1.0;            % m/s, 1-sigma intensity per axis
p.wind.turb_tau    = 2.0;            % s, correlation time

% =====================================================================
% Lead compensators (NON-PX4 addition).
% Setpoint pre-filters used to compensate the ramp-tracking lag inherent
% to the cascaded P/PID controller. Each channel implements the
% first-order lead H(s) = (Ts*s + 1) / (Tp*s + 1) with Ts > Tp:
%   - DC gain is 1, so static setpoints pass through unchanged.
%   - Max phase lead   = asin((Ts - Tp) / (Ts + Tp))
%   - Centre frequency = 1 / sqrt(Ts*Tp)  [rad/s]
% Disable a stage by setting <stage>.enable = false. Reset is performed
% by run_interactive on Reset and on every mode change so the filter
% state cannot kick when the setpoint jumps.
% =====================================================================
% Position setpoint lead (per [N; E; D] axis).
%   xy : centre ~0.7 Hz, max phase ~33 deg
%   z  : centre ~1.1 Hz, max phase ~33 deg
p.lead.pos.enable = true;
p.lead.pos.Ts = [0.40; 0.40; 0.25];
p.lead.pos.Tp = [0.12; 0.12; 0.08];

% Velocity feedforward lead (per [N; E; D] axis).
%   xy : centre ~1.4 Hz, max phase ~25 deg
%   z  : centre ~2.1 Hz, max phase ~25 deg
p.lead.vel.enable = true;
p.lead.vel.Ts = [0.18; 0.18; 0.12];
p.lead.vel.Tp = [0.072; 0.072; 0.048];

% Attitude setpoint lead (roll, pitch only). Yaw is intentionally
% omitted: a lead filter on a +/-pi-wrapping signal would inject
% spurious transients on wrap-around. Yaw is passed through untouched.
%   roll/pitch : centre ~3.7 Hz, max phase ~20 deg
p.lead.att.enable = true;
p.lead.att.Ts = [0.06; 0.06];
p.lead.att.Tp = [0.03; 0.03];

% =====================================================================
% Iris airframe (plant). Source: iris.sdf.
% =====================================================================
p.airframe.mass = 1.5;       % kg, base_link mass
% Body inertia tensor (kg*m^2). Diagonal; rotor parallel-axis contribution
% (~1e-3 on Ixx/Iyy) is small and absorbed by controller robustness.
p.airframe.I = diag([0.029125, 0.029125, 0.055225]);

% Rotor positions in FRD body frame (meters), Iris geometry.
% iris.sdf rotor poses (in ENU body, x-fwd, y-left, z-up):
%   rotor_0 (FR): ( 0.13, -0.22, 0.023)
%   rotor_1 (BL): (-0.13,  0.20, 0.023)
%   rotor_2 (FL): ( 0.13,  0.22, 0.023)
%   rotor_3 (BR): (-0.13, -0.20, 0.023)
% Convert to FRD: x_FRD = x_ENU, y_FRD = -y_ENU, z_FRD = -z_ENU.
p.airframe.rotor_pos_frd = [ ...
    0.13,  0.22, -0.023;   % rotor 0 FR
   -0.13, -0.20, -0.023;   % rotor 1 BL
    0.13, -0.22, -0.023;   % rotor 2 FL
   -0.13,  0.20, -0.023]'; % rotor 3 BR
% (Stored 3x4 so each column is one rotor's position vector.)

% Per-rotor max thrust (N). iris.sdf: motorConstant = 5.84e-6,
% maxRotVelocity = 1100 rad/s -> T_max = 5.84e-6 * 1100^2 = 7.0664 N.
p.airframe.rotor_thrust_max = 5.84e-6 * 1100^2;
% Per-rotor reaction-torque coefficient = momentConstant * motorConstant
% (iris.sdf: momentConstant = 0.06). Per-rotor max torque = km * T_max.
% This 0.06 corresponds to PX4 CA_ROTORn_KM = 0.05 in the unit-allocator;
% the slight mismatch is an Iris-vs-generic-quad detail and is small.
p.airframe.rotor_moment_const = 0.06;

% Spin direction per rotor (+1 = CW from above = +z reaction in FRD,
% -1 = CCW). Matches PX4 quad-X km signs.
p.airframe.rotor_spin = [+1; +1; -1; -1];

% Convenience: total max thrust and hover ratio sanity check.
%   T_total_max = 4 * 7.0664 = 28.27 N
%   T_hover     = m*g = 14.715 N -> hover ratio = 0.520 (matches MPC_THR_HOVER=0.5)

end
