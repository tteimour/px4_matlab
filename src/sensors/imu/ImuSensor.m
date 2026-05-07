classdef ImuSensor < Sensor
% ImuSensor — abstract base for 6-DOF IMU chips (gyro + accel).
%
% Concrete chip subclasses (ICM45686, IIM42652, ADIS16470) set:
%   * sample_rate_hz  — chip ODR
%   * latency_s       — driver / bus latency
%   * gyro_fs_dps     — gyro full scale (deg/s)
%   * accel_fs_g      — accel full scale (g)
%   * gyro_quant_dps  — quantisation step (dps/LSB)
%   * accel_quant_g   — quantisation step (g/LSB)
%   * gyro_nd         — gyro noise density (rad/s/sqrt(Hz))
%   * accel_nd        — accel noise density (m/s^2/sqrt(Hz))
%   * gyro_brw        — gyro bias random walk (rad/s/sqrt(s))
%   * accel_brw       — accel bias random walk (m/s^2/sqrt(s))
%   * gyro_turn_on    — turn-on bias 1-sigma (rad/s)
%   * accel_turn_on   — turn-on bias 1-sigma (m/s^2)
%   * R_chip_to_body  — 3x3 rotation matrix (chip frame to FRD body)
%
% measure() outputs sample struct with fields:
%   t           sample timestamp (s)
%   gyro_b      [3x1] body-frame gyro reading (rad/s) after rotation
%   accel_b     [3x1] body-frame specific force (m/s^2)
%   delta_ang   [3x1] body-frame integrated angle over sample period (rad)
%                — matches PX4 imu_sample.delta_ang (ekf.cpp)
%   delta_vel   [3x1] body-frame integrated specific force * dt (m/s)
%                — matches PX4 imu_sample.delta_vel
%   delta_ang_dt, delta_vel_dt  scalar dt for the deltas (s)
%   instance, device_id
%
% PX4 reference for the delta-form IMU sample:
%   src/modules/ekf2/EKF/ekf.cpp predictState() lines 231-277

    properties
        gyro_fs_dps
        accel_fs_g
        gyro_quant_dps
        accel_quant_g
        gyro_nd
        accel_nd
        gyro_brw
        accel_brw
        gyro_turn_on
        accel_turn_on
        R_chip_to_body
        earth                % EarthModel handle (for gravity)
    end

    properties (Access = protected)
        gyro_bias_           % current bias [3x1] rad/s
        accel_bias_          % current bias [3x1] m/s^2
        last_t_              % previous sample time for delta integration
    end

    methods
        function obj = ImuSensor(rate_hz, latency_s, priority, instance, ...
                                 device_id, earth)
            obj@Sensor(rate_hz, latency_s, priority, instance, device_id);
            obj.earth = earth;
            obj.R_chip_to_body = eye(3);
            obj.gyro_bias_  = zeros(3, 1);
            obj.accel_bias_ = zeros(3, 1);
            obj.last_t_     = 0.0;
        end

        function initBias(obj)
            % Sample turn-on biases. Call after subclass constructor sets
            % the sigma values.
            obj.gyro_bias_  = obj.gyro_turn_on  * randn(obj.rng, 3, 1);
            obj.accel_bias_ = obj.accel_turn_on * randn(obj.rng, 3, 1);
        end

        function reset(obj)
            reset@Sensor(obj);
            obj.last_t_ = 0.0;
            obj.initBias();
        end
    end

    methods (Access = protected)
        function s = measure(obj, t_sample, gt)
            % gt fields used:
            %   gt.angular_vel_b   [3x1] body-frame rates (rad/s)
            %   gt.acceleration_ned [3x1] inertial accel of body in NED (m/s^2)
            %   gt.attitude_q       [4x1] body->NED quaternion [w;x;y;z]
            dt = max(t_sample - obj.last_t_, obj.sample_period_);
            obj.last_t_ = t_sample;

            % Bias random walk update.
            obj.gyro_bias_  = obj.gyro_bias_  + obj.gyro_brw  * sqrt(dt) * randn(obj.rng, 3, 1);
            obj.accel_bias_ = obj.accel_bias_ + obj.accel_brw * sqrt(dt) * randn(obj.rng, 3, 1);

            % --- Gyro: body angular rate in chip frame, biased + noisy + quantised.
            omega_body  = gt.angular_vel_b;
            omega_chip  = obj.R_chip_to_body' * omega_body;
            gyro_noise  = obj.gyro_nd / sqrt(dt) * randn(obj.rng, 3, 1);
            gyro_chip   = omega_chip + obj.R_chip_to_body' * obj.gyro_bias_ + gyro_noise;
            gyro_chip   = obj.quantise(gyro_chip, deg2rad(obj.gyro_quant_dps));
            gyro_chip   = obj.clipFs(gyro_chip, deg2rad(obj.gyro_fs_dps));
            gyro_b      = obj.R_chip_to_body * gyro_chip;

            % --- Accel: specific force = a_inertial - g, expressed in body.
            R_b2n   = quat_to_dcm(gt.attitude_q);
            R_n2b   = R_b2n';
            g_ned   = obj.earth.gravityNed();
            sf_body = R_n2b * (gt.acceleration_ned - g_ned);
            sf_chip = obj.R_chip_to_body' * sf_body;
            sf_noise = obj.accel_nd / sqrt(dt) * randn(obj.rng, 3, 1);
            sf_chip = sf_chip + obj.R_chip_to_body' * obj.accel_bias_ + sf_noise;
            sf_chip = obj.quantise(sf_chip, obj.accel_quant_g * obj.earth.g_mps2);
            sf_chip = obj.clipFs(sf_chip, obj.accel_fs_g * obj.earth.g_mps2);
            accel_b = obj.R_chip_to_body * sf_chip;

            % Delta-angle / delta-velocity match PX4 imu_sample.
            s.gyro_b       = gyro_b;
            s.accel_b      = accel_b;
            s.delta_ang    = gyro_b  * dt;
            s.delta_vel    = accel_b * dt;
            s.delta_ang_dt = dt;
            s.delta_vel_dt = dt;
        end

        function v = quantise(~, v, step)
            if step > 0
                v = round(v / step) * step;
            end
        end

        function v = clipFs(~, v, fs)
            v = max(min(v, fs), -fs);
        end
    end
end
