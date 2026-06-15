classdef Ekf2 < handle
% Ekf2 — 24-state error-state EKF, subset replica of PX4's EKF2.
%
% Reference: src/modules/ekf2/EKF/{ekf.cpp, covariance.cpp,
% aid_sources/{baro_height,gnss,magnetometer,gravity}/}
%
% State (24 components) — order matches PX4 state.h:
%   [1:4]   quat     body->NED quaternion [w; x; y; z]
%   [5:7]   vel      NED velocity (m/s)
%   [8:10]  pos      NED position (m)
%   [11:13] gyro_b   gyro bias (rad/s)
%   [14:16] accel_b  accel bias (m/s^2)
%   [17:19] mag_I    earth magnetic field, NED (Gauss)
%   [20:22] mag_B    body magnetic bias (Gauss)
%   [23:24] wind     horizontal wind N/E (m/s)
%
% Error-state covariance is 23x23; the attitude block is a 3-component
% rotation-vector (small-angle) error, all other states are linear.
% Error-state index map (P rows/cols):
%   [1:3]   attitude error
%   [4:6]   velocity error
%   [7:9]   position error
%   [10:12] gyro bias error
%   [13:15] accel bias error
%   [16:18] mag_I error
%   [19:21] mag_B error
%   [22:23] wind error
%
% Fusions implemented (subset chosen with the user):
%   * predict (IMU)              ekf.cpp:231-277, covariance.cpp:113-200
%   * baro height                aid_sources/baro_height/baro_height_control.cpp
%   * GNSS pos (h+v)             aid_sources/gnss/gps_control.cpp
%   * GNSS vel                   aid_sources/gnss/gps_control.cpp
%   * mag 3D                     aid_sources/magnetometer/mag_fusion.cpp:53-141
%   * mag heading                aid_sources/magnetometer/mag_control.cpp:136-150
%   * gravity                    aid_sources/gravity/gravity_fusion.cpp
%
% Not implemented (deferred, per CLAUDE.md scope): optical flow, range,
% airspeed, sideslip, drag, external vision, terrain.

    properties
        params       % Ekf2Params struct
        earth        % EarthModel handle (used for gravity, mag_I init)

        % Nominal state.
        quat
        vel
        pos
        gyro_b
        accel_b
        mag_I
        mag_B
        wind

        % Error covariance (23x23).
        P

        % Bookkeeping.
        initialized      = false
        last_imu_t       = 0.0
        gnss_origin_set  = false
        % Baro height-bias estimator — port of PX4 BiasEstimator
        % (bias_estimator.{hpp,cpp}): constant-state KF with a small
        % process PSD so the bias learns over minutes, an innovation gate,
        % and variance limits. (The previous ad-hoc filter had a ~0.3 s
        % learning time constant and no gate: on the ground it wound up
        % ~14 m of phantom bias within seconds, then froze, leaving the
        % baro permanently gate-rejected and the height channel adrift.)
        baro_bias        = 0.0
        baro_bias_var    = 0.1    % initial variance (bias_estimator.hpp:104)
        last_baro_fuse_t = -inf

        % VIO irtifa-ofset kestiricisi (#5): monoküler VIO'nun düşey
        % sürüklenmesini soğurur; böylece gevşek (ev_p_noise_z) VIO-z,
        % baro ile çapalanan irtifayı bozmaz. Baro bias ile aynı desen.
        % Gözlemlenebilir: dondurulmuş baro mutlak irtifayı sabitlerken bu
        % ofset VIO-z sürüklenmesini üstlenir. (Yalnızca düşey — yatay VIO
        % ofseti GNSS/TRN olmadan gözlemlenemez, eklenmedi.)
        vio_z_bias        = 0.0
        vio_z_bias_var    = 1.0
        last_vio_z_fuse_t = -inf

        % GNSS yardımı aktif mi? VIO (GNSS erişimsiz) modunda false olur;
        % o zaman baro bias DONDURULUR (yeniden öğrenecek mutlak referans
        % yok), böylece baro kararlı mutlak irtifa referansı olarak kalır.
        gnss_active      = true

        % IMU downsampler (imu_down_sampler.cpp): raw 1 kHz IMU samples are
        % accumulated into one EKF2_PREDICT_US window (default 10 ms); the
        % state + covariance prediction runs once per window, like PX4.
        ds_dq            = [1; 0; 0; 0]   % accumulated delta-angle quaternion
        ds_dvel          = zeros(3, 1)    % accumulated delta velocity (rotated frame)
        ds_dt            = 0.0            % accumulated window time
        ds_target_dt     = 0.010          % s, from params.predict_us
        dt_ekf           = 0.010          % dt of the last completed EKF update
        ds_sf_body       = zeros(3, 1)    % specific force of the last window (gravity fusion)

        % Control-status flags fed by the vehicle layer (PX4 control.cpp):
        % in_air gates mag-3D alignment; at_rest relaxes the gravity-fusion
        % accel-magnitude window (gravity_fusion.cpp:61-63); ev_active marks
        % external vision as the horizontal aid (isHorizontalAidingActive).
        in_air               = false
        at_rest              = true
        ev_active            = false
        mag_aligned_in_flight = false

        % Filter initialisation (Ekf::initialiseFilter, ekf.cpp:180-229):
        % the EKF refuses to predict or fuse until the smoothed specific
        % force is within [0.8 g, 1.2 g] and the gyro below 15 deg/s; tilt
        % is then initialised from the measured gravity direction and the
        % height state from the first baro sample, so the first fusions
        % see small innovations instead of a violent cold-start transient.
        filter_init      = false
        init_windows_ok  = 0      % consecutive downsampled windows passing checks
        init_baro_alt    = []     % latest baro altitude (set by EstimatorBus)
    end

    properties (Constant)
        IDX_ATT  = 1:3
        IDX_VEL  = 4:6
        IDX_POS  = 7:9
        IDX_GBI  = 10:12
        IDX_ABI  = 13:15
        IDX_MI   = 16:18
        IDX_MB   = 19:21
        IDX_WIND = 22:23
        N_ERR    = 23
    end

    methods
        function obj = Ekf2(params, earth)
            obj.params = params;
            obj.earth  = earth;
            obj.reset();
        end

        function reset(obj)
            obj.quat    = [1; 0; 0; 0];
            obj.vel     = zeros(3, 1);
            obj.pos     = zeros(3, 1);
            obj.gyro_b  = zeros(3, 1);
            obj.accel_b = zeros(3, 1);
            obj.mag_I   = obj.earth.magNed();
            obj.mag_B   = zeros(3, 1);
            obj.wind    = zeros(2, 1);

            obj.initCovariance();

            obj.initialized     = false;
            obj.last_imu_t      = 0.0;
            obj.gnss_origin_set = false;
            obj.baro_bias       = 0.0;
            obj.baro_bias_var   = 0.1;
            obj.last_baro_fuse_t = -inf;
            obj.vio_z_bias       = 0.0;
            obj.vio_z_bias_var   = 1.0;
            obj.last_vio_z_fuse_t = -inf;
            obj.gnss_active      = true;

            obj.ds_dq         = [1; 0; 0; 0];
            obj.ds_dvel       = zeros(3, 1);
            obj.ds_dt         = 0.0;
            obj.ds_target_dt  = double(obj.params.predict_us) * 1e-6;
            obj.dt_ekf        = obj.ds_target_dt;
            obj.ds_sf_body    = zeros(3, 1);
            obj.in_air                = false;
            obj.at_rest               = true;
            obj.ev_active             = false;
            obj.mag_aligned_in_flight = false;

            obj.filter_init     = false;
            obj.init_windows_ok = 0;
            obj.init_baro_alt   = [];
        end

        function setInitBaro(obj, altitude_m)
            % Latest baro altitude for height initialisation (EstimatorBus
            % feeds this until the filter initialises).
            obj.init_baro_alt = altitude_m;
        end

        % ============================================================
        % Predict: accumulate raw IMU samples (imu_down_sampler.cpp:13-52)
        % and run one state + covariance prediction per EKF2_PREDICT_US
        % window (estimator_interface.cpp / ekf.cpp predictState). Returns
        % true when an EKF update ran (callers gate fusions on this).
        % ============================================================
        function updated = predict(obj, imu)
            updated = false;
            dt_s = max(imu.delta_ang_dt, 1e-6);

            % --- accumulate (imu_down_sampler.cpp:25-39) ---------------
            ang = norm(imu.delta_ang);
            if ang > 1e-9
                axn = imu.delta_ang / ang;
                dq  = [cos(ang/2); axn * sin(ang/2)];
            else
                dq  = [1; 0.5 * imu.delta_ang];
            end
            obj.ds_dq = quat_multiply(obj.ds_dq, dq);
            obj.ds_dq = obj.ds_dq / norm(obj.ds_dq);
            % rotate accumulated delta-vel into the new frame, then add the
            % new sample assuming it spans the rotation half-way
            R_delta  = quat_to_dcm(dq)';                 % Dcm(delta_q.inversed())
            obj.ds_dvel = R_delta * obj.ds_dvel ...
                        + (imu.delta_vel + R_delta * imu.delta_vel) * 0.5;
            obj.ds_dt = obj.ds_dt + dt_s;

            obj.last_imu_t = imu.t;
            if obj.ds_dt < obj.ds_target_dt - 0.5 * dt_s
                return;                                  % window not full yet
            end

            % --- finalize window -> one EKF prediction -----------------
            dt = obj.ds_dt;
            qv = obj.ds_dq;
            sang = norm(qv(2:4));
            angw = 2 * atan2(sang, qv(1));
            if sang > 1e-12
                d_ang_w = qv(2:4) / sang * angw;         % axis-angle vector
            else
                d_ang_w = 2 * qv(2:4);
            end
            d_vel_w = obj.ds_dvel;
            obj.ds_dq   = [1; 0; 0; 0];
            obj.ds_dvel = zeros(3, 1);
            obj.ds_dt   = 0.0;
            obj.dt_ekf  = dt;
            obj.ds_sf_body = d_vel_w / dt;               % for gravity fusion

            % No prediction or fusion until the filter has initialised
            % (Ekf::initialiseFilter, ekf.cpp:180-211).
            if ~obj.filter_init
                obj.tryInitFilter(d_ang_w / dt, d_vel_w / dt);
                return;
            end

            % Bias-corrected delta angle / velocity.
            d_ang = d_ang_w - obj.gyro_b * dt;
            d_vel = d_vel_w - obj.accel_b * dt;

            % --- Quaternion propagation: q_new = q ⊗ exp(0.5 * d_ang) ---
            ang   = norm(d_ang);
            if ang > 1e-9
                ax = d_ang / ang;
                dq = [cos(ang/2); ax * sin(ang/2)];
            else
                dq = [1; 0.5 * d_ang];
            end
            obj.quat = quat_multiply(obj.quat, dq);
            obj.quat = obj.quat / norm(obj.quat);

            R_b2n  = quat_to_dcm(obj.quat);
            d_vel_n = R_b2n * d_vel;
            g_ned  = obj.earth.gravityNed();

            vel_old = obj.vel;
            obj.vel = vel_old + d_vel_n + g_ned * dt;
            obj.pos = obj.pos + 0.5 * (vel_old + obj.vel) * dt;

            obj.predictCovariance(d_ang, d_vel, dt, R_b2n);
            obj.constrainStateVariances();

            obj.initialized = true;
            updated = true;
        end

        % ============================================================
        % Initial covariance (Ekf::initialiseCovariance,
        % covariance.cpp:52-104): vertical velocity gets 1.5^2x the
        % horizontal variance, height variance comes from the baro.
        % Called on reset and again when the filter initialises.
        % ============================================================
        function initCovariance(obj)
            obj.P = zeros(obj.N_ERR);
            p = obj.params;
            obj.P(obj.IDX_ATT,  obj.IDX_ATT)  = eye(3) * p.init_att_var;
            obj.P(obj.IDX_VEL,  obj.IDX_VEL)  = diag([1, 1, 1.5^2] * p.init_vel_var);
            obj.P(obj.IDX_POS,  obj.IDX_POS)  = diag([p.init_pos_var, p.init_pos_var, ...
                                                      max(p.baro_noise, 0.01)^2]);
            obj.P(obj.IDX_GBI,  obj.IDX_GBI)  = eye(3) * p.init_gyro_b_var;
            obj.P(obj.IDX_ABI,  obj.IDX_ABI)  = eye(3) * p.init_accel_b_var;
            obj.P(obj.IDX_MI,   obj.IDX_MI)   = eye(3) * p.init_mag_I_var;
            obj.P(obj.IDX_MB,   obj.IDX_MB)   = eye(3) * p.init_mag_B_var;
            obj.P(obj.IDX_WIND, obj.IDX_WIND) = eye(2) * p.init_wind_var;
        end

        % ============================================================
        % Diagonal variance limiting (covariance.cpp:244-263, constants
        % ekf.h:442-446). Last-resort guard against runaway Kalman gains.
        % ============================================================
        function constrainStateVariances(obj)
            obj.clampDiag(obj.IDX_ATT,  1e-9, 1.0);
            obj.clampDiag(obj.IDX_VEL,  1e-6, 1e6);
            obj.clampDiag(obj.IDX_POS,  1e-6, 1e6);
            obj.clampDiag(obj.IDX_GBI,  1e-9, 1.0);   % kGyroBiasVarianceMin
            obj.clampDiag(obj.IDX_ABI,  1e-9, 1.0);   % kAccelBiasVarianceMin
            obj.clampDiag(obj.IDX_MI,   1e-6, 1.0);   % kMagVarianceMin
            obj.clampDiag(obj.IDX_MB,   1e-6, 1.0);
            obj.clampDiag(obj.IDX_WIND, 1e-6, 1e6);
        end

        function predictCovariance(obj, d_ang, d_vel, dt, R_b2n)
            % Build continuous-time error-state Jacobian F (23x23).
            F = zeros(obj.N_ERR);

            omega = d_ang / dt;       % corrected angular vel
            a_b   = d_vel / dt;       % corrected specific force, body

            F(obj.IDX_ATT, obj.IDX_ATT) = -skew(omega);
            F(obj.IDX_ATT, obj.IDX_GBI) = -eye(3);

            F(obj.IDX_VEL, obj.IDX_ATT) = -R_b2n * skew(a_b);
            F(obj.IDX_VEL, obj.IDX_ABI) = -R_b2n;

            F(obj.IDX_POS, obj.IDX_VEL) = eye(3);

            % First-order discretisation.
            F_d = eye(obj.N_ERR) + F * dt;

            % Process noise.
            p = obj.params;
            Q = zeros(obj.N_ERR);
            Q(obj.IDX_ATT,  obj.IDX_ATT)  = eye(3) * (p.gyr_noise^2)   * dt;
            Q(obj.IDX_VEL,  obj.IDX_VEL)  = R_b2n * eye(3) * (p.acc_noise^2) * dt * R_b2n';
            Q(obj.IDX_GBI,  obj.IDX_GBI)  = eye(3) * (p.gyr_b_noise^2) * dt;
            Q(obj.IDX_ABI,  obj.IDX_ABI)  = eye(3) * (p.acc_b_noise^2) * dt;
            Q(obj.IDX_MI,   obj.IDX_MI)   = eye(3) * (p.mag_e_noise^2) * dt;
            Q(obj.IDX_MB,   obj.IDX_MB)   = eye(3) * (p.mag_b_noise^2) * dt;
            Q(obj.IDX_WIND, obj.IDX_WIND) = eye(2) * (p.wind_nsd^2)    * dt;

            obj.P = F_d * obj.P * F_d' + Q;
            obj.P = 0.5 * (obj.P + obj.P');     % symmetrise
        end

        % ============================================================
        % Baro fusion — vertical position update
        % baro_height_control.cpp lines 82-126
        % ============================================================
        function out = fuseBaro(obj, baro)
            out.fused = false; out.innov = 0; out.test_ratio = 0;
            if isempty(baro), return; end

            % Predict the height-bias variance first (BiasEstimator::predict,
            % bias_estimator.cpp:42-63): PSD = baro_bias_nsd^2 = 0.13^2
            % (common.h:304), variance constrained to [1e-8, 2].
            if isfinite(obj.last_baro_fuse_t)
                dt_b = max(baro.t - obj.last_baro_fuse_t, 0);
            else
                dt_b = 0;
            end
            obj.last_baro_fuse_t = baro.t;
            % Bias varyansını YALNIZCA GNSS mutlak irtifayı çapalarken büyüt.
            % VIO (GNSS erişimsiz) modunda baro tek mutlak irtifa referansıdır;
            % bias dondurulur ki baro kararlı kalsın.
            if obj.gnss_active
                obj.baro_bias_var = min(max(obj.baro_bias_var + 0.13^2 * dt_b, 1e-8), 2.0);
            end

            % Measurement: altitude (positive up). Predicted altitude = alt0 - pos_z.
            % The observation variance includes the bias-estimator variance
            % (baro_height_control.cpp:84-86) — as the bias uncertainty grows
            % the baro is de-weighted in the main fusion, so the (unbiased)
            % GNSS height anchors the DC level while the bias state absorbs
            % the baro's turn-on offset and drift.
            pred_alt = obj.earth.alt0_m - obj.pos(3);
            innov    = baro.altitude_m - pred_alt - obj.baro_bias;

            H = zeros(1, obj.N_ERR);
            H(obj.IDX_POS(3)) = -1.0;     % d(alt)/d(pos_z) = -1

            R = obj.params.baro_noise^2 + obj.baro_bias_var;
            S = H * obj.P * H' + R;
            test_ratio = innov^2 / (S * obj.params.baro_innov_gate^2);

            out.innov = innov; out.test_ratio = test_ratio; out.S = S;
            if test_ratio > 1.0, return; end

            obj.applyKalmanUpdate(H, innov, S, R);

            % Height-bias state update (BiasEstimator::fuseBias,
            % bias_estimator.cpp:70-86): the bias innovation is
            % (baro_alt - est_alt) - bias = the SAME residual as the main
            % fusion innovation above (PX4 fuseBias(measurement -
            % gpos.altitude()) with the state subtracted internally).
            % Sign matters: bias += K*innov closes the residual (negative
            % feedback). The previous filter used -innov, which drives the
            % bias AWAY from the residual; the main fusion then drags the
            % height state after it — a positive-feedback pair that ran
            % the altitude estimate away at ~2 m/s. Innovation variance
            % includes the filter's own height variance
            % (gnss_height_control.cpp:45 pattern), 3-sigma gate (hpp:103).
            innov_var_b = obj.baro_bias_var + max(1e-4, obj.params.baro_noise^2) ...
                        + obj.P(obj.IDX_POS(3), obj.IDX_POS(3));
            innov_b     = innov;
            % Bias yalnızca GNSS aktifken öğrenilir; VIO modunda dondurulur.
            if obj.gnss_active && innov_b^2 / (9 * innov_var_b) < 1.0
                K_b = obj.baro_bias_var / innov_var_b;
                obj.baro_bias     = obj.baro_bias + K_b * innov_b;
                obj.baro_bias_var = max((1 - K_b) * obj.baro_bias_var, 1e-8);
            end
            out.fused = true;
        end

        % ============================================================
        % GNSS position fusion (3-axis, NED)
        % ============================================================
        function out = fuseGnssPos(obj, gps)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(gps), return; end

            % First-fix initialisation: seed EKF position from the GPS
            % report in the existing shared earth frame. Do NOT mutate
            % the EarthModel — sensor models depend on its origin being
            % stable for the lifetime of the sim.
            if ~obj.gnss_origin_set
                obj.pos = obj.earth.llaToNed(gps.lat_deg, gps.lon_deg, gps.alt_m);
                obj.gnss_origin_set = true;
                return;
            end

            z = obj.earth.llaToNed(gps.lat_deg, gps.lon_deg, gps.alt_m);

            % Observation noise floored by the receiver-reported accuracy
            % (gps_control.cpp:219 pattern: max(reported, param)). The M9N
            % model reports eph/epv with each sample.
            r_h = obj.params.gps_p_noise;
            r_v = 1.5 * obj.params.gps_p_noise;
            if isfield(gps, 'eph'), r_h = max(r_h, gps.eph); end
            if isfield(gps, 'epv'), r_v = max(r_v, gps.epv); end
            R_pos = diag([r_h^2, r_h^2, r_v^2]);

            for axis = 1:3
                innov = z(axis) - obj.pos(axis);
                H = zeros(1, obj.N_ERR);
                H(obj.IDX_POS(axis)) = 1.0;
                S = H * obj.P * H' + R_pos(axis, axis);
                tr = innov^2 / (S * obj.params.gps_pos_gate^2);
                out.innov(axis)      = innov;
                out.test_ratio(axis) = tr;
                if tr <= 1.0
                    obj.applyKalmanUpdate(H, innov, S, R_pos(axis, axis));
                end
            end
            out.fused = true;
        end

        % ============================================================
        % GNSS velocity fusion (3-axis, NED)
        % ============================================================
        function out = fuseGnssVel(obj, gps)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(gps), return; end

            % Observation noise floored by the reported speed accuracy
            % (gps_control.cpp:219: max(gnss_sample.sacc, gps_vel_noise)).
            % The GnssSensor sample carries sacc^2 as s_variance_mps2.
            r_v = obj.params.gps_v_noise;
            if isfield(gps, 's_variance_mps2')
                r_v = max(r_v, sqrt(gps.s_variance_mps2));
            end
            R_vel = diag([r_v^2, r_v^2, (r_v * 1.5)^2]);

            for axis = 1:3
                innov = gps.vel_ned(axis) - obj.vel(axis);
                H = zeros(1, obj.N_ERR);
                H(obj.IDX_VEL(axis)) = 1.0;
                S = H * obj.P * H' + R_vel(axis, axis);
                tr = innov^2 / (S * obj.params.gps_vel_gate^2);
                out.innov(axis)      = innov;
                out.test_ratio(axis) = tr;
                if tr <= 1.0
                    obj.applyKalmanUpdate(H, innov, S, R_vel(axis, axis));
                end
            end
            out.fused = true;
        end

        % ============================================================
        % External-vision (VIO) position fusion (3-axis, NED).
        % Same sequential per-axis Kalman update as fuseGnssPos, but the
        % measurement `z_ned` is already in the EKF NED frame -- the caller
        % anchors the OpenVINS `global` frame to the EKF state when the VIO
        % aid is enabled (yaw+origin are otherwise unobservable). Noise from
        % EKF2_EVP_NOISE, gate from EKF2_EVP_GATE (params_external_vision.yaml).
        % ============================================================
        function out = fuseVioPos(obj, z_ned, t)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(z_ned), return; end
            if nargin < 3 || isempty(t), t = obj.last_vio_z_fuse_t; end

            % Eksen-bazlı gürültü: yatay sıkı (VIO tek yatay yardım), düşey
            % gevşek (ev_p_noise_z) -> mutlak irtifayı baro tutar, VIO-z yalnızca
            % iter. Düşey eksende VIO irtifa-ofseti (#5) ölçümden çıkarılır;
            % böylece monoküler VIO düşey sürüklenmesi irtifaya değil ofsete biner.
            R_axis = [obj.params.ev_p_noise^2, obj.params.ev_p_noise^2, ...
                      obj.params.ev_p_noise_z^2];

            % --- VIO irtifa-ofset varyansının rastgele yürüyüşle büyümesi ---
            if isfinite(obj.last_vio_z_fuse_t)
                dt_v = max(t - obj.last_vio_z_fuse_t, 0);
            else
                dt_v = 0;
            end
            obj.vio_z_bias_var = min(obj.vio_z_bias_var + obj.params.vio_z_bias_nsd^2 * dt_v, 50.0);
            obj.last_vio_z_fuse_t = t;

            for axis = 1:3
                if axis == 3
                    % VIO düşey ölçümü = pos_d + vio_z_bias (+gürültü).
                    innov = z_ned(3) - obj.pos(3) - obj.vio_z_bias;
                    R     = R_axis(3) + obj.vio_z_bias_var;
                else
                    innov = z_ned(axis) - obj.pos(axis);
                    R     = R_axis(axis);
                end
                H = zeros(1, obj.N_ERR);
                H(obj.IDX_POS(axis)) = 1.0;
                S = H * obj.P * H' + R;
                tr = innov^2 / (S * obj.params.ev_pos_gate^2);
                out.innov(axis)      = innov;
                out.test_ratio(axis) = tr;
                if tr <= 1.0
                    obj.applyKalmanUpdate(H, innov, S, R);
                    if axis == 3
                        % VIO irtifa-ofsetini aynı artıkla güncelle (baro bias
                        % deseni): ofset, VIO-z sürüklenmesini üstlenir.
                        innov_var_b = obj.vio_z_bias_var + R_axis(3) ...
                                    + obj.P(obj.IDX_POS(3), obj.IDX_POS(3));
                        if innov^2 / (9 * innov_var_b) < 1.0
                            K_b = obj.vio_z_bias_var / innov_var_b;
                            obj.vio_z_bias     = obj.vio_z_bias + K_b * innov;
                            obj.vio_z_bias_var = max((1 - K_b) * obj.vio_z_bias_var, 1e-6);
                        end
                    end
                end
            end
            out.fused = true;
        end

        % VIO füzyonu açıldığında irtifa-ofsetini sıfırla (anchor anında
        % VIO-z ile EKF irtifası çakışık olduğundan ofset 0'dan başlar).
        function resetVioHeightBias(obj)
            obj.vio_z_bias        = 0.0;
            obj.vio_z_bias_var    = 1.0;
            obj.last_vio_z_fuse_t = -inf;
        end

        % ============================================================
        % External-vision (VIO) velocity fusion (3-axis, NED).
        % `v_ned` must already be rotated into the EKF NED frame by the
        % caller. Noise EKF2_EVV_NOISE, gate EKF2_EVV_GATE.
        % ============================================================
        function out = fuseVioVel(obj, v_ned)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(v_ned), return; end

            R_vel = (obj.params.ev_v_noise^2) * eye(3);
            for axis = 1:3
                innov = v_ned(axis) - obj.vel(axis);
                H = zeros(1, obj.N_ERR);
                H(obj.IDX_VEL(axis)) = 1.0;
                S = H * obj.P * H' + R_vel(axis, axis);
                tr = innov^2 / (S * obj.params.ev_vel_gate^2);
                out.innov(axis)      = innov;
                out.test_ratio(axis) = tr;
                if tr <= 1.0
                    obj.applyKalmanUpdate(H, innov, S, R_vel(axis, axis));
                end
            end
            out.fused = true;
        end

        % ============================================================
        % Mag 3D fusion — sequential per-axis update
        % mag_fusion.cpp:53-141. update_tilt mirrors fuseMag(...,
        % update_all_states, update_tilt): when false the Kalman-gain rows
        % of the roll/pitch attitude error are zeroed (mag_fusion.cpp:
        % 107-111), so the mag cannot pull the tilt — PX4 only enables
        % tilt updates from mag (mag_3D) after the in-flight alignment
        % (mag_control.cpp:186-192).
        % ============================================================
        function out = fuseMag3D(obj, mag_sample, update_tilt)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(mag_sample), return; end
            if nargin < 3, update_tilt = true; end

            if update_tilt
                K_zero = [];
            else
                K_zero = obj.IDX_ATT(1:2);    % zero roll/pitch error gains
            end

            R_b2n = quat_to_dcm(obj.quat);
            R_n2b = R_b2n';

            for axis = 1:3
                pred = R_n2b(axis, :) * obj.mag_I + obj.mag_B(axis);
                innov = mag_sample.mag_b(axis) - pred;

                H = zeros(1, obj.N_ERR);
                % d(pred)/d(δθ) for body-frame attitude error δθ:
                %   q_pert = q ⊗ exp(δθ/2)  =>  R_b2n_pert = R_b2n*(I+skew(δθ))
                %   R_n2b_pert = (I - skew(δθ))*R_n2b
                %   pred_pert = R_n2b_pert * mag_I = Rm - skew(δθ)*Rm
                %             = Rm + skew(Rm)*δθ      (cross identity)
                %   so d(pred)/dδθ = +skew(Rm).
                Rm = R_n2b * obj.mag_I;
                S_skew = skew(Rm);
                H(obj.IDX_ATT) = S_skew(axis, :);
                H(obj.IDX_MI)  = R_n2b(axis, :);
                H(obj.IDX_MB(axis)) = 1.0;

                Rm_var = obj.params.mag_noise^2;
                S = H * obj.P * H' + Rm_var;
                tr = innov^2 / (S * obj.params.mag_gate^2);
                out.innov(axis)      = innov;
                out.test_ratio(axis) = tr;
                if tr <= 1.0
                    obj.applyKalmanUpdate(H, innov, S, Rm_var, K_zero);
                end
            end
            out.fused = true;
        end

        % ============================================================
        % Mag heading fusion — scalar yaw update from horizontal mag
        % mag_control.cpp:136-150
        % ============================================================
        function out = fuseMagHeading(obj, mag_sample)
            out.fused = false; out.innov = 0; out.test_ratio = 0;
            if isempty(mag_sample), return; end

            R_b2n   = quat_to_dcm(obj.quat);
            mag_n   = R_b2n * (mag_sample.mag_b - obj.mag_B);
            measured_yaw = atan2(mag_n(2), mag_n(1)) - obj.params.mag_decl;

            eul = quat_to_euler(obj.quat);
            pred_yaw = eul(3);
            innov = wrap_pi(measured_yaw - pred_yaw);

            H = zeros(1, obj.N_ERR);
            H(obj.IDX_ATT(3)) = 1.0;       % yaw error ≈ δθ_z

            Rm_var = obj.params.head_noise^2;
            S = H * obj.P * H' + Rm_var;
            tr = innov^2 / (S * obj.params.hdg_gate^2);
            out.innov = innov; out.test_ratio = tr; out.S = S;
            if tr <= 1.0
                obj.applyKalmanUpdate(H, innov, S, Rm_var);
                out.fused = true;
            end
        end

        % ============================================================
        % Gravity fusion — accel-as-gravity in low-acceleration flight
        % gravity_fusion.cpp:49-115. Gating per lines 54-63: needs the
        % accel magnitude within [0.9 g, 1.1 g] OR vehicle at rest, and
        % NO active horizontal aiding (GNSS or external vision) — with a
        % biased accel, gravity fusion would otherwise pull the attitude
        % to "absorb" the bias, locking in a tilt error.
        % Uses the specific force of the last downsampled EKF window
        % (ds_sf_body), so it runs once per EKF update like PX4.
        % ============================================================
        function out = fuseGravity(obj)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);

            % isHorizontalAidingActive() equivalent (gravity_fusion.cpp:63).
            gnss_aiding = obj.gnss_origin_set && bitand(obj.params.gps_ctrl, 1) ~= 0;
            if gnss_aiding || obj.ev_active
                return;
            end

            R_b2n = quat_to_dcm(obj.quat);
            R_n2b = R_b2n';

            % Accel-magnitude window (gravity_fusion.cpp:55-58): 0.9..1.1 g,
            % bypassed when the vehicle is at rest.
            sf = obj.ds_sf_body - obj.accel_b;
            nsf = norm(sf);
            accel_norm_good = (nsf > 0.9 * obj.earth.g_mps2) && ...
                              (nsf < 1.1 * obj.earth.g_mps2);
            if ~(accel_norm_good || obj.at_rest)
                return;
            end

            measurement = sf / max(norm(sf), 1e-3);
            predicted   = R_n2b * [0; 0; -1];     % expected -z body, since gravity is +z NED

            % Standard innovation convention: z - h(x). All other
            % fusions in this class use the same convention.
            innov = measurement - predicted;

            for axis = 1:3
                H = zeros(1, obj.N_ERR);
                S_skew = skew(R_n2b * [0; 0; -1]);
                H(obj.IDX_ATT) = S_skew(axis, :);

                Rg_var = (0.1 * obj.earth.g_mps2)^2;
                S = H * obj.P * H' + Rg_var;
                tr = innov(axis)^2 / (S * obj.params.gravity_gate^2);
                out.innov(axis)      = innov(axis);
                out.test_ratio(axis) = tr;
                if tr <= 1.0
                    obj.applyKalmanUpdate(H, innov(axis), S, Rg_var);
                end
            end
            out.fused = true;
        end

        % ============================================================
        % Flight-phase flags from the vehicle layer (PX4 control.cpp gets
        % these from commander / land detector). in_air enables mag-3D
        % tilt updates: PX4 sets mag_aligned_in_flight at the first
        % in-flight mag alignment (mag_control.cpp:186-192); the sim's mag
        % prior is the exact earth field, so alignment is implicit when
        % entering flight. at_rest = ~in_air is a sim simplification of
        % PX4's dedicated at-rest detector.
        % ============================================================
        function setFlightPhase(obj, in_air)
            obj.in_air  = logical(in_air);
            obj.at_rest = ~obj.in_air;
            if obj.in_air
                obj.mag_aligned_in_flight = true;
            end
        end

        % ============================================================
        % Output struct (controller-facing): same shape as the plant
        % ground-truth struct so the controller is unchanged.
        % ============================================================
        function s = stateOut(obj, omega_meas)
            s.position_ned     = obj.pos;
            s.velocity_ned     = obj.vel;
            s.attitude_q       = obj.quat;
            if nargin >= 2 && ~isempty(omega_meas)
                s.angular_vel_b = omega_meas - obj.gyro_b;
            else
                s.angular_vel_b = zeros(3, 1);
            end
            R_b2n = quat_to_dcm(obj.quat);
            s.acceleration_ned = R_b2n * (-obj.accel_b) + obj.earth.gravityNed();
        end
    end

    methods (Access = private)
        function applyKalmanUpdate(obj, H, innov, S, R, K_zero)
            % K_zero (optional): error-state indices whose Kalman gain is
            % forced to zero before the update — PX4 does this to keep a
            % fusion from updating specific states (mag_fusion.cpp:107-122).
            % The Joseph-form covariance update below stays consistent for
            % any (suboptimal) gain.
            P_local = obj.P;
            K = (P_local * H') / S;
            if nargin >= 6 && ~isempty(K_zero)
                K(K_zero) = 0;
            end

            dx = K * innov;        % 23x1 error-state correction

            % Inject attitude error as a small-angle quaternion.
            dtheta = dx(obj.IDX_ATT);
            ang = norm(dtheta);
            if ang > 1e-9
                ax = dtheta / ang;
                dq = [cos(ang/2); ax * sin(ang/2)];
            else
                dq = [1; 0.5 * dtheta];
            end
            obj.quat = quat_multiply(obj.quat, dq);
            obj.quat = obj.quat / norm(obj.quat);

            % State updates with PX4's hard limits (Ekf::fuse,
            % ekf_helper.cpp:750-791). The bias clamps are essential: in
            % static/hover flight {tilt, accel bias, mag states} share an
            % unobservable subspace, and without the +/-0.4 m/s^2 accel
            % bias limit (common.h:472) the filter can settle into a
            % self-consistent wrong equilibrium (several degrees of tilt
            % "explained" by a phantom bias).
            pl = obj.params;
            obj.vel     = clampv(obj.vel     + dx(obj.IDX_VEL), 1e3);
            obj.pos     = obj.pos + dx(obj.IDX_POS);
            obj.gyro_b  = clampv(obj.gyro_b  + dx(obj.IDX_GBI), pl.gyro_bias_lim);
            obj.accel_b = clampv(obj.accel_b + dx(obj.IDX_ABI), pl.acc_bias_lim);
            obj.mag_I   = clampv(obj.mag_I   + dx(obj.IDX_MI), 1.0);
            obj.mag_B   = clampv(obj.mag_B   + dx(obj.IDX_MB), pl.mag_bias_lim);
            obj.wind    = clampv(obj.wind    + dx(obj.IDX_WIND), 100);

            % Joseph-form covariance update (numerically stable).
            I_KH = eye(obj.N_ERR) - K * H;
            obj.P = I_KH * P_local * I_KH' + K * R * K';
            obj.P = 0.5 * (obj.P + obj.P');
        end

        function clampDiag(obj, idx, lo, hi)
            % Clamp P diagonal entries of one state group (covariance.cpp
            % constrainStateVar).
            for i = idx
                obj.P(i, i) = min(max(obj.P(i, i), lo), hi);
            end
        end

        function tryInitFilter(obj, omega, sf)
            % Ekf::initialiseFilter / initialiseTilt (ekf.cpp:180-229):
            % static checks on the downsampled window (its 10 ms average
            % stands in for PX4's accel/gyro low-pass; three consecutive
            % windows required), then tilt from the measured gravity
            % direction and height from the latest baro sample.
            g = obj.earth.g_mps2;
            ok = (norm(sf) > 0.8 * g) && (norm(sf) < 1.2 * g) && ...
                 (norm(omega) < deg2rad(15));                  % ekf.cpp:218-220
            if ~ok
                obj.init_windows_ok = 0;
                return;
            end
            obj.init_windows_ok = obj.init_windows_ok + 1;
            if obj.init_windows_ok < 3 || isempty(obj.init_baro_alt)
                return;
            end
            % Tilt: quaternion rotating the measured specific force onto
            % [0 0 -1] (initialiseTilt, ekf.cpp:224-226).
            obj.quat = quat_from_two_vectors(sf, [0; 0; -1]);
            % Height from baro (pred_alt = alt0 - pos_z); horizontal stays
            % at the origin until the first GNSS fix seeds it.
            obj.pos = [0; 0; obj.earth.alt0_m - obj.init_baro_alt];
            obj.vel = zeros(3, 1);
            obj.initCovariance();                              % ekf.cpp:205
            obj.filter_init = true;
        end
    end
end


function q = quat_from_two_vectors(v1, v2)
% Quaternion rotating v1 onto v2 (matrix::Quaternion(v1, v2), used by
% Ekf::initialiseTilt). Handles the antiparallel case explicitly.
v1 = v1 / max(norm(v1), 1e-9);
v2 = v2 / max(norm(v2), 1e-9);
d = dot(v1, v2);
if d < -1 + 1e-9
    [~, i] = min(abs(v1));
    e = zeros(3, 1); e(i) = 1;
    ax = cross(v1, e); ax = ax / norm(ax);
    q = [0; ax];
else
    q = [1 + d; cross(v1, v2)];
    q = q / norm(q);
end
end


function v = clampv(v, lim)
% Symmetric per-component clamp (matrix::constrain in Ekf::fuse).
v = min(max(v, -lim), lim);
end
