classdef OutputPredictor < handle
% OutputPredictor — IMU-rate strapdown with complementary correction
% toward the (possibly delayed) EKF state.
%
% Reference: src/modules/ekf2/EKF/output_predictor/output_predictor.cpp
%   calculateOutputStates() lines 175-257     IMU-rate strapdown
%   correctOutputStates()   lines 259-348     PI feedback to buffer
%   applyCorrectionToOutputBuffer() line 385
%
% This MATLAB version implements the corrected-output behaviour
% (complementary filter against the EKF output) but skips the ring
% buffer / delay-aligned correction loop. Adequate for sims where the
% IMU and EKF run at the same rate.

    properties
        quat
        vel
        pos
        gyro_b
        accel_b
        earth
        tau_vel
        tau_pos
        time_delay      % output_predictor time horizon (params.out_time_delay)
        last_t

        % Correction state (output_predictor.cpp:259-348). delta_ang_corr is
        % computed once per EKF update and applied to EVERY subsequent IMU
        % strapdown step (calculateOutputStates adds _delta_angle_corr each
        % sample); vel/pos PI corrections apply once per EKF update.
        delta_ang_corr = zeros(3, 1)
        vel_err_integ  = zeros(3, 1)
        pos_err_integ  = zeros(3, 1)
        dt_update_avg  = 1e-3       % average IMU strapdown dt
    end

    methods
        function obj = OutputPredictor(params, earth)
            obj.earth      = earth;
            obj.tau_vel    = params.tau_vel;
            obj.tau_pos    = params.tau_pos;
            obj.time_delay = params.out_time_delay;
            obj.reset();
        end

        function reset(obj)
            obj.quat   = [1; 0; 0; 0];
            obj.vel    = zeros(3, 1);
            obj.pos    = zeros(3, 1);
            obj.gyro_b = zeros(3, 1);
            obj.accel_b= zeros(3, 1);
            obj.last_t = 0.0;
            obj.delta_ang_corr = zeros(3, 1);
            obj.vel_err_integ  = zeros(3, 1);
            obj.pos_err_integ  = zeros(3, 1);
            obj.dt_update_avg  = 1e-3;
        end

        function alignTo(obj, ekf)
            % Match the EKF state at filter initialisation
            % (OutputPredictor::alignOutputFilter, called from
            % Ekf::initialiseFilter, ekf.cpp:208).
            obj.quat    = ekf.quat;
            obj.vel     = ekf.vel;
            obj.pos     = ekf.pos;
            obj.gyro_b  = ekf.gyro_b;
            obj.accel_b = ekf.accel_b;
            obj.delta_ang_corr = zeros(3, 1);
            obj.vel_err_integ  = zeros(3, 1);
            obj.pos_err_integ  = zeros(3, 1);
        end

        function update(obj, imu)
            dt = max(imu.delta_ang_dt, 1e-6);
            obj.dt_update_avg = 0.8 * obj.dt_update_avg + 0.2 * dt;

            % Attitude correction feed: the per-EKF-update delta-angle
            % correction is folded into each strapdown step
            % (output_predictor.cpp calculateOutputStates: delta_angle +=
            % _delta_angle_corr).
            d_ang = imu.delta_ang - obj.gyro_b * dt + obj.delta_ang_corr;
            d_vel = imu.delta_vel - obj.accel_b * dt;

            ang = norm(d_ang);
            if ang > 1e-9
                ax = d_ang / ang;
                dq = [cos(ang/2); ax * sin(ang/2)];
            else
                dq = [1; 0.5 * d_ang];
            end
            obj.quat = quat_multiply(obj.quat, dq);
            obj.quat = obj.quat / norm(obj.quat);

            R_b2n   = quat_to_dcm(obj.quat);
            d_vel_n = R_b2n * d_vel;
            g_ned   = obj.earth.gravityNed();

            vel_old = obj.vel;
            obj.vel = vel_old + d_vel_n + g_ned * dt;
            obj.pos = obj.pos + 0.5 * (vel_old + obj.vel) * dt;

            obj.last_t = imu.t;
        end

        function correctTo(obj, ekf, dt_correct)
            % Called once per EKF update (PX4 correctOutputStates,
            % output_predictor.cpp:259-348). dt_correct = EKF update dt.
            obj.gyro_b  = ekf.gyro_b;
            obj.accel_b = ekf.accel_b;
            dt_correct = min(max(dt_correct, 1e-4), 0.03);   % cpp:265

            % Attitude (output_predictor.cpp:287-302): q_error =
            % q_state^-1 * q_out is the BODY-frame error of the output
            % w.r.t. the EKF; delta_ang_error = -2*imag(q_error) (sign-
            % normalized) is the body-frame rotation that pulls the output
            % onto the EKF attitude, applied by update() on every
            % strapdown step until the next correction. The error MUST be
            % composed in the body frame because it is applied through
            % the body delta-angle path — a world-frame error here works
            % at yaw ~ 0 but mixes the roll/pitch axes at large yaw and
            % destabilizes the correction loop.
            % att_gain = 0.5 * dt_imu / time_delay; because the correction
            % is applied (dt_correct / dt_imu) times per cycle, time_delay
            % is floored at the CORRECTION interval — otherwise the total
            % per-cycle correction exceeds 100% of the error and the
            % output attitude oscillates. (On PX4 hardware the real
            % delayed-horizon depth is always >= the update interval, so
            % its fmaxf(..., dt_update) floor never binds; this floor is
            % the zero-latency-sim generalization of the same guard.)
            q_err = quat_multiply(quat_inverse(ekf.quat), obj.quat);
            q_err = q_err / norm(q_err);
            if q_err(1) >= 0, scalar = -2; else, scalar = 2; end   % cpp:290
            delta_ang_err = scalar * q_err(2:4);
            td = max(obj.time_delay, dt_correct);
            att_gain = 0.5 * obj.dt_update_avg / td;
            obj.delta_ang_corr = delta_ang_err * att_gain;

            % Velocity / position: proportional + weak integral tracker
            % (cpp:305-341). Gains from the correction interval and the
            % vel/pos time constants.
            vel_gain = dt_correct / min(max(obj.tau_vel, dt_correct), 10.0);
            pos_gain = dt_correct / min(max(obj.tau_pos, dt_correct), 10.0);
            vel_err = ekf.vel - obj.vel;
            pos_err = ekf.pos - obj.pos;
            obj.vel_err_integ = obj.vel_err_integ + vel_err;
            obj.pos_err_integ = obj.pos_err_integ + pos_err;
            obj.vel = obj.vel + vel_err * vel_gain ...
                    + obj.vel_err_integ * (vel_gain^2) * 0.1;
            obj.pos = obj.pos + pos_err * pos_gain ...
                    + obj.pos_err_integ * (pos_gain^2) * 0.1;
        end

        function s = stateOut(obj, omega_meas)
            s.position_ned = obj.pos;
            s.velocity_ned = obj.vel;
            s.attitude_q   = obj.quat;
            if nargin >= 2 && ~isempty(omega_meas)
                s.angular_vel_b = omega_meas - obj.gyro_b;
            else
                s.angular_vel_b = zeros(3, 1);
            end
            R_b2n = quat_to_dcm(obj.quat);
            s.acceleration_ned = R_b2n * (-obj.accel_b) + obj.earth.gravityNed();
        end
    end
end
