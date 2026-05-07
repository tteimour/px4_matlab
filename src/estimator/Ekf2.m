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
        baro_bias        = 0.0    % running baro bias (HeightBiasEstimator equivalent)
        baro_bias_var    = 1.0
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

            obj.P = zeros(obj.N_ERR);
            p = obj.params;
            obj.P(obj.IDX_ATT,  obj.IDX_ATT)  = eye(3) * p.init_att_var;
            obj.P(obj.IDX_VEL,  obj.IDX_VEL)  = eye(3) * p.init_vel_var;
            obj.P(obj.IDX_POS,  obj.IDX_POS)  = eye(3) * p.init_pos_var;
            obj.P(obj.IDX_GBI,  obj.IDX_GBI)  = eye(3) * p.init_gyro_b_var;
            obj.P(obj.IDX_ABI,  obj.IDX_ABI)  = eye(3) * p.init_accel_b_var;
            obj.P(obj.IDX_MI,   obj.IDX_MI)   = eye(3) * p.init_mag_I_var;
            obj.P(obj.IDX_MB,   obj.IDX_MB)   = eye(3) * p.init_mag_B_var;
            obj.P(obj.IDX_WIND, obj.IDX_WIND) = eye(2) * p.init_wind_var;

            obj.initialized     = false;
            obj.last_imu_t      = 0.0;
            obj.gnss_origin_set = false;
            obj.baro_bias       = 0.0;
            obj.baro_bias_var   = 1.0;
        end

        % ============================================================
        % Predict (called per IMU sample)
        % ============================================================
        function predict(obj, imu)
            % imu is a vehicle_imu sample with delta_ang, delta_vel, dt fields.
            dt = max(imu.delta_ang_dt, 1e-6);

            % Bias-corrected delta angle / velocity.
            d_ang = imu.delta_ang - obj.gyro_b * dt;
            d_vel = imu.delta_vel - obj.accel_b * dt;

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

            obj.last_imu_t = imu.t;
            obj.initialized = true;
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

            % Measurement: altitude (positive up). Predicted altitude = alt0 - pos_z.
            pred_alt = obj.earth.alt0_m - obj.pos(3);
            innov    = baro.altitude_m - pred_alt - obj.baro_bias;

            H = zeros(1, obj.N_ERR);
            H(obj.IDX_POS(3)) = -1.0;     % d(alt)/d(pos_z) = -1

            R = obj.params.baro_noise^2;
            S = H * obj.P * H' + R;
            test_ratio = innov^2 / (S * obj.params.baro_innov_gate^2);

            out.innov = innov; out.test_ratio = test_ratio; out.S = S;
            if test_ratio > 1.0, return; end

            obj.applyKalmanUpdate(H, innov, S, R);

            % Slow bias estimator (matches PX4 HeightBiasEstimator).
            K_b = obj.baro_bias_var / (obj.baro_bias_var + R);
            obj.baro_bias     = obj.baro_bias + K_b * (-innov);
            obj.baro_bias_var = (1 - K_b) * obj.baro_bias_var + 1e-4;
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

            R_pos = diag([obj.params.gps_p_noise^2, ...
                          obj.params.gps_p_noise^2, ...
                          (obj.params.gps_p_noise * 1.5)^2]);

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

            R_vel = diag([obj.params.gps_v_noise^2, ...
                          obj.params.gps_v_noise^2, ...
                          (obj.params.gps_v_noise * 1.5)^2]);

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
        % Mag 3D fusion — sequential per-axis update
        % mag_fusion.cpp:53-141
        % ============================================================
        function out = fuseMag3D(obj, mag_sample)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(mag_sample), return; end

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
                    obj.applyKalmanUpdate(H, innov, S, Rm_var);
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
        % gravity_fusion.cpp:49-115
        % ============================================================
        function out = fuseGravity(obj, imu)
            out.fused = false; out.innov = zeros(3, 1); out.test_ratio = zeros(3, 1);
            if isempty(imu), return; end

            R_b2n = quat_to_dcm(obj.quat);
            R_n2b = R_b2n';

            % Reject if specific force not close to 1g.
            sf = imu.accel_b - obj.accel_b;
            if abs(norm(sf) - obj.earth.g_mps2) > 0.5 * obj.earth.g_mps2
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
        function applyKalmanUpdate(obj, H, innov, S, R)
            P_local = obj.P;
            K = (P_local * H') / S;

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

            obj.vel     = obj.vel     + dx(obj.IDX_VEL);
            obj.pos     = obj.pos     + dx(obj.IDX_POS);
            obj.gyro_b  = obj.gyro_b  + dx(obj.IDX_GBI);
            obj.accel_b = obj.accel_b + dx(obj.IDX_ABI);
            obj.mag_I   = obj.mag_I   + dx(obj.IDX_MI);
            obj.mag_B   = obj.mag_B   + dx(obj.IDX_MB);
            obj.wind    = obj.wind    + dx(obj.IDX_WIND);

            % Joseph-form covariance update (numerically stable).
            I_KH = eye(obj.N_ERR) - K * H;
            obj.P = I_KH * P_local * I_KH' + K * R * K';
            obj.P = 0.5 * (obj.P + obj.P');
        end
    end
end
