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

    % --- External vision / VIO (params_external_vision.yaml) ---
    % Used when the VIO-tab "Fuse VIO -> EKF" toggle replaces GNSS with
    % OpenVINS odometry as the position/velocity aiding source.
    p.ev_p_noise       = 0.1;        % m    (EKF2_EVP_NOISE default)
    p.ev_v_noise       = 0.1;        % m/s  (EKF2_EVV_NOISE default)
    p.ev_pos_gate      = 5.0;        % STD  (EKF2_EVP_GATE default)
    p.ev_vel_gate      = 3.0;        % STD  (EKF2_EVV_GATE default)

    % --- Wind ---
    p.wind_nsd         = 1.0e-1;     % m/s/sqrt(s) wind process noise

    % --- Output predictor (params_core.yaml) ---
    p.tau_vel          = 0.25;       % s
    p.tau_pos          = 0.25;       % s

    % --- Initial covariance scales (state.h equivalent diag init) ---
    p.init_att_var     = (deg2rad(10))^2;        % rad^2
    p.init_vel_var     = 1.0^2;                  % (m/s)^2
    p.init_pos_var     = 1.0^2;                  % m^2
    p.init_gyro_b_var  = (deg2rad(2))^2;         % (rad/s)^2 — wide enough
                                                  % for ~0.035 rad/s bias.
    p.init_accel_b_var = 0.5^2;                  % (m/s^2)^2 — accel bias
                                                  % is poorly observable in
                                                  % hover; leave the prior
                                                  % wide so GNSS-vel can
                                                  % pull the estimate to
                                                  % truth via P_vel_abi
                                                  % cross-covariance.
    p.init_mag_I_var   = 0.05^2;                 % G^2
    p.init_mag_B_var   = 0.02^2;                 % G^2
    p.init_wind_var    = 1.0^2;                  % (m/s)^2

    % --- Misc ---
    p.imu_ctrl         = 7;          % bitmask: 1 gyro_bias, 2 accel_bias, 4 gravity
    p.gravity_gate     = 0.25;       % gravity fusion gate (hardcoded in PX4)
end
