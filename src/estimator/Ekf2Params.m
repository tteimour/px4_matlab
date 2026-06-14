function p = Ekf2Params()
% Ekf2Params — default EKF2 parameters mirroring PX4 EKF2_* names.
%
% Sourced from src/modules/ekf2/params_*.yaml in the PX4 reference tree.
% Use this struct verbatim when constructing an Ekf2 instance; override
% individual fields after the call to tune for your sim.
%
% Naming convention: lower-case fields without the EKF2_ prefix, so
% EKF2_GYR_NOISE -> p.gyr_noise, EKF2_GPS_P_NOISE -> p.gps_p_noise, etc.

    % --- IMU process noise (params_core.yaml) ---
    p.gyr_noise        = 1.5e-2;     % rad/s
    p.acc_noise        = 3.5e-1;     % m/s^2
    p.gyr_b_noise      = 1.0e-3;     % rad/s^2 (bias random walk power)
    p.acc_b_noise      = 3.0e-3;     % m/s^3

    % --- Magnetometer (params_mag.yaml) ---
    p.mag_e_noise      = 1.0e-3;     % Gauss/s   earth-field process noise
    p.mag_b_noise      = 1.0e-4;     % Gauss/s   body-bias process noise
    p.head_noise       = 0.3;        % rad       heading meas noise
    p.mag_noise        = 5.0e-2;     % Gauss     3D-mag meas noise
    p.mag_decl         = 0.0;        % rad       initial declination
    p.mag_gate         = 5.0;        % STD       3D-mag innovation gate
    p.hdg_gate         = 2.6;        % STD       heading gate
    p.mag_type         = 0;          % 0=auto, 1=heading, 2=3D, 3=none, 4=init

    % --- Barometer (params_baro.yaml) ---
    p.baro_noise       = 2.0;        % m
    p.baro_innov_gate  = 5.0;        % STD
    p.baro_delay       = 0.0;        % s

    % --- GNSS (params_gnss.yaml) ---
    p.gps_p_noise      = 0.5;        % m
    p.gps_v_noise      = 0.3;        % m/s
    p.gps_pos_gate     = 5.0;        % STD
    p.gps_vel_gate     = 5.0;        % STD
    p.gps_delay        = 0.110;      % s
    p.gps_ctrl         = 7;          % bitmask: 1 long+lat, 2 alt, 4 vel
    p.req_eph          = 3.0;
    p.req_epv          = 5.0;
    p.req_sacc         = 0.5;
    p.req_nsats        = 6;

    % --- Wind ---
    p.wind_nsd         = 1.0e-1;     % m/s/sqrt(s) wind process noise

    % --- Filter update scheduling (module.yaml EKF2_PREDICT_US) ---
    % PX4 downsamples the raw IMU stream (imu_down_sampler.cpp) and runs
    % the state + covariance prediction once per accumulated window of
    % EKF2_PREDICT_US (default 10000 us -> 100 Hz), NOT per IMU sample.
    p.predict_us       = 10000;      % us  (EKF2_PREDICT_US default)

    % --- Output predictor (params_core.yaml) ---
    p.tau_vel          = 0.25;       % s
    p.tau_pos          = 0.25;       % s
    % Output-predictor attitude correction time horizon. PX4 computes
    % att_gain = 0.5 * dt_imu / time_delay (output_predictor.cpp:297-298)
    % where time_delay = now - delayed fusion horizon. On real hardware
    % that is ~the largest aiding delay (gps_delay_ms = 110 ms,
    % EKF/common.h:326). THIS SIM generates sensor samples with zero
    % fusion latency, so the horizon is 0 and the formula degenerates to
    % att_gain = 0.5 per IMU sample — the output tracks the EKF attitude
    % tightly, exactly as PX4 would with an empty delay buffer.
    p.out_time_delay   = 0.0;        % s (zero-latency sim horizon)

    % --- Controller-facing gyro low-pass (PX4 VehicleAngularVelocity) ---
    % The rate the controller regulates = bias-corrected gyro, low-pass
    % filtered. PX4 filters the gyro (IMU_GYRO_CUTOFF, default 40 Hz;
    % imu_gyro_parameters.c:128) before mc_rate_control; the EKF prediction
    % still uses the RAW gyro. 0 disables the filter (raw gyro to the rate loop).
    p.gyro_cutoff_hz   = 40.0;       % Hz   (IMU_GYRO_CUTOFF default)

    % --- Initial covariance (Ekf::initialiseCovariance, covariance.cpp:52-104) ---
    % The sim spawns with the quaternion exactly level, so PX4's
    % resetQuatCov(0.f) "no initial uncertainty" applies directly
    % (covariance.cpp:56). Velocity/position priors are seeded from the
    % aiding noise like PX4; bias priors use the switch-on 1-sigmas
    % (common.h:296-297). The previously hand-widened priors (att (0.1)^2,
    % accel bias 0.5^2) let the filter slide along the {tilt, accel bias,
    % mag} unobservable subspace into a tilted equilibrium — the state
    % clamps in Ekf2.applyKalmanUpdate plus these PX4 priors remove that.
    p.init_att_var     = 0.0;                    % rad^2 (resetQuatCov(0.f))
    p.init_vel_var     = max(p.gps_v_noise, 0.01)^2;   % covariance.cpp:60-64
                                                        % (z gets 1.5^2x in Ekf2.reset)
    p.init_pos_var     = max(p.gps_p_noise, 0.01)^2;   % covariance.cpp:74 (xy;
                                                        % z uses baro_noise^2, :68)
    p.init_gyro_b_var  = 0.1^2;                  % (rad/s)^2, switch_on_gyro_bias
                                                  % (common.h:296, covariance.cpp:337)
    p.init_accel_b_var = 0.2^2;                  % (m/s^2)^2, switch_on_accel_bias
                                                  % (common.h:297, covariance.cpp:349)
    p.init_mag_I_var   = p.mag_noise^2;          % G^2 (covariance.cpp:357)
    p.init_mag_B_var   = p.mag_noise^2;          % G^2 (covariance.cpp:364)
    p.init_wind_var    = 1.0^2;                  % (m/s)^2

    % --- State hard limits (Ekf::fuse, ekf_helper.cpp:750-791) ---
    p.gyro_bias_lim    = 0.4;        % rad/s  (common.h:477)
    p.acc_bias_lim     = 0.4;        % m/s^2  (common.h:472)
    p.mag_bias_lim     = 0.5;        % Gauss  (ekf.h:284 getMagBiasLimit)

    % --- Misc ---
    p.imu_ctrl         = 7;          % bitmask: 1 gyro_bias, 2 accel_bias, 4 gravity
    p.gravity_gate     = 0.25;       % gravity fusion gate (hardcoded in PX4)
end
