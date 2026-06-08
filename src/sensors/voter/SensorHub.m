classdef SensorHub < handle
% SensorHub — top-level sensor manager.
%
% Owns one VotedSensors group per sensor type (IMU, baro, mag, GNSS).
% Steps every sensor each sim tick and exposes the voted primary
% samples in PX4-style structs:
%
%   vehicle_imu         (gyro_b, accel_b, delta_ang/dt, delta_vel/dt, t)
%   vehicle_air_data    (altitude_m, pressure_pa, temperature_c, t)
%   vehicle_magnetometer(mag_b, t)
%   vehicle_gps_position(lat, lon, alt, vel_ned, eph, epv, fix_type, t)
%   sensor_selection    (device IDs of the selected primaries)
%
% Reference for the published struct shapes:
%   src/modules/sensors/voted_sensors_update.cpp  (sensor_selection_s)
%   src/modules/sensors/vehicle_imu/VehicleIMU.cpp
%   src/modules/sensors/vehicle_air_data/VehicleAirData.cpp
%   src/modules/sensors/vehicle_magnetometer/VehicleMagnetometer.cpp
%   src/modules/sensors/vehicle_gps_position/VehicleGPSPosition.cpp

    properties
        imu      % VotedSensors
        baro     % VotedSensors
        mag      % VotedSensors
        gnss     % VotedSensors
        earth    % EarthModel
    end

    methods
        function obj = SensorHub(earth, varargin)
            % Optional: SensorHub(earth, 'imus', {...}, 'baros', {...}, ...).
            % If a list is omitted, build the default V6X_6 stack.
            obj.earth = earth;

            opts = struct('imus', {{}}, 'baros', {{}}, 'mags', {{}}, 'gnss', {{}});
            for k = 1:2:numel(varargin)
                opts.(lower(varargin{k})) = varargin{k+1};
            end

            if isempty(opts.imus)
                opts.imus = { ImuICM45686([], [], earth), ...
                              ImuIIM42652([], [], earth), ...
                              ImuADIS16470([], [], earth) };
            end
            if isempty(opts.baros)
                opts.baros = { BaroICP201XX([], [], earth), ...
                               BaroBMP388([], [], earth) };
            end
            if isempty(opts.mags)
                opts.mags  = { MagBMM150([], [], earth), ...
                               MagIST8310([], [], earth) };
            end
            if isempty(opts.gnss)
                opts.gnss  = { GnssM9N([], [], earth) };
            end

            obj.imu  = VotedSensors(opts.imus);
            obj.baro = VotedSensors(opts.baros);
            obj.mag  = VotedSensors(opts.mags);
            obj.gnss = VotedSensors(opts.gnss);
        end

        function step(obj, t, ground_truth)
            obj.imu.step(t, ground_truth);
            obj.baro.step(t, ground_truth);
            obj.mag.step(t, ground_truth);
            obj.gnss.step(t, ground_truth);
        end

        function out = sensorSelection(obj)
            out.timestamp        = 0;
            out.accel_device_id  = obj.imu.primaryDeviceId();
            out.gyro_device_id   = obj.imu.primaryDeviceId();
            out.baro_device_id   = obj.baro.primaryDeviceId();
            out.mag_device_id    = obj.mag.primaryDeviceId();
            out.gps_device_id    = obj.gnss.primaryDeviceId();
        end

        function s = vehicleImu(obj),         s = obj.imu.primary();  end
        function s = vehicleAirData(obj),     s = obj.baro.primary(); end
        function s = vehicleMagnetometer(obj),s = obj.mag.primary();  end
        function s = vehicleGpsPosition(obj), s = obj.gnss.primary(); end

        function applyImuBias(obj, gyro_b, accel_b)
            % Set the same body-frame bias on every IMU instance — used
            % for EKF bias-estimation tests where the sim needs a known
            % "truth" bias to plot against.
            for i = 1:numel(obj.imu.sensors)
                obj.imu.sensors{i}.setBias(gyro_b, accel_b);
            end
        end

        function [g, a] = primaryImuTrueBias(obj)
            % Live body-frame bias of the currently-voted primary IMU
            % (includes any bias-random-walk drift accumulated since
            % construction).
            idx = obj.imu.selected_idx;
            if idx > 0
                g = obj.imu.sensors{idx}.trueGyroBias();
                a = obj.imu.sensors{idx}.trueAccelBias();
            else
                g = zeros(3, 1); a = zeros(3, 1);
            end
        end

        function reset(obj)
            % Reset every sensor group (clocks, delay queues, validators,
            % primary selection) back to its initial state.
            obj.imu.reset();
            obj.baro.reset();
            obj.mag.reset();
            obj.gnss.reset();
        end
    end
end
