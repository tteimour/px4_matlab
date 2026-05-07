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
        att_gain
        last_t
    end

    methods
        function obj = OutputPredictor(params, earth)
            obj.earth    = earth;
            obj.tau_vel  = params.tau_vel;
            obj.tau_pos  = params.tau_pos;
            obj.att_gain = 0.5;     % PX4 uses 0.5 * dt_imu / time_delay
            obj.reset();
        end

        function reset(obj)
            obj.quat   = [1; 0; 0; 0];
            obj.vel    = zeros(3, 1);
            obj.pos    = zeros(3, 1);
            obj.gyro_b = zeros(3, 1);
            obj.accel_b= zeros(3, 1);
            obj.last_t = 0.0;
        end

        function update(obj, imu)
            dt = max(imu.delta_ang_dt, 1e-6);

            d_ang = imu.delta_ang - obj.gyro_b * dt;
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

        function correctTo(obj, ekf, dt_imu)
            % Pull toward the EKF state with PI-style gains.
            obj.gyro_b  = ekf.gyro_b;
            obj.accel_b = ekf.accel_b;

            % Attitude correction: small-angle representation of q_ekf * q_out^-1.
            q_err = quat_multiply(ekf.quat, quat_inverse(obj.quat));
            q_err = q_err / norm(q_err);
            if q_err(1) < 0, q_err = -q_err; end
            ang   = 2 * atan2(norm(q_err(2:4)), q_err(1));
            if ang > 1e-6
                axis = q_err(2:4) / norm(q_err(2:4));
            else
                axis = [0; 0; 0];
            end
            dtheta = axis * ang * obj.att_gain;
            ang_c  = norm(dtheta);
            if ang_c > 1e-9
                ax = dtheta / ang_c;
                dq = [cos(ang_c/2); ax * sin(ang_c/2)];
            else
                dq = [1; 0.5 * dtheta];
            end
            obj.quat = quat_multiply(dq, obj.quat);
            obj.quat = obj.quat / norm(obj.quat);

            % Velocity / position complementary filter.
            k_v = min(dt_imu / max(obj.tau_vel, 1e-3), 1.0);
            k_p = min(dt_imu / max(obj.tau_pos, 1e-3), 1.0);
            obj.vel = obj.vel + k_v * (ekf.vel - obj.vel);
            obj.pos = obj.pos + k_p * (ekf.pos - obj.pos);
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
