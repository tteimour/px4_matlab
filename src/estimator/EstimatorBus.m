classdef EstimatorBus < handle
% EstimatorBus — top-level orchestration of sensors + EKF2 + output predictor.
%
% Sits between QuadrotorDynamics (ground truth) and the controller.
% Each sim step:
%   1. SensorHub.step(t, ground_truth) — generates noisy samples,
%      runs voter, updates vehicle_* primary publications.
%   2. If a new vehicle_imu sample is published: Ekf2.predict() and
%      OutputPredictor.update().
%   3. Run available aiding fusions when their corresponding voted
%      sample is fresh:
%         vehicle_air_data       -> fuseBaro
%         vehicle_gps_position   -> fuseGnssPos + fuseGnssVel
%         vehicle_magnetometer   -> fuseMag3D (or fuseMagHeading per
%                                  EKF2_MAG_TYPE)
%         low-acceleration       -> fuseGravity
%   4. After the fusion(s), OutputPredictor.correctTo(ekf) pulls the
%      high-rate output toward the corrected EKF state.
%   5. stateOut() returns the controller-facing state struct.
%
% Sample staleness is tracked by remembering each voted sample's last
% consumed timestamp; a fusion only fires when the timestamp advances.

    properties
        sensors      % SensorHub
        ekf          % Ekf2
        output_pred  % OutputPredictor
        params

        last_imu_t   = -inf
        last_baro_t  = -inf
        last_mag_t   = -inf
        last_gps_t   = -inf
    end

    methods
        function obj = EstimatorBus(earth, varargin)
            obj.params      = Ekf2Params();
            obj.sensors     = SensorHub(earth, varargin{:});
            obj.ekf         = Ekf2(obj.params, earth);
            obj.output_pred = OutputPredictor(obj.params, earth);
        end

        function step(obj, t, ground_truth)
            obj.sensors.step(t, ground_truth);

            imu = obj.sensors.vehicleImu();
            if ~isempty(imu) && isfield(imu, 't') && imu.t > obj.last_imu_t
                obj.ekf.predict(imu);
                obj.output_pred.update(imu);
                obj.last_imu_t = imu.t;
            end

            % Fusion order matters: height first, then position/vel, then mag.
            air = obj.sensors.vehicleAirData();
            if ~isempty(air) && isfield(air, 't') && air.t > obj.last_baro_t
                obj.ekf.fuseBaro(air);
                obj.last_baro_t = air.t;
            end

            gps = obj.sensors.vehicleGpsPosition();
            if ~isempty(gps) && isfield(gps, 't') && gps.t > obj.last_gps_t ...
                    && bitand(obj.params.gps_ctrl, 1) ~= 0
                obj.ekf.fuseGnssPos(gps);
                if bitand(obj.params.gps_ctrl, 4) ~= 0
                    obj.ekf.fuseGnssVel(gps);
                end
                obj.last_gps_t = gps.t;
            end

            mag = obj.sensors.vehicleMagnetometer();
            if ~isempty(mag) && isfield(mag, 't') && mag.t > obj.last_mag_t
                if obj.params.mag_type == 1
                    obj.ekf.fuseMagHeading(mag);
                else
                    obj.ekf.fuseMag3D(mag);
                end
                obj.last_mag_t = mag.t;
            end

            % Gravity fusion if accel ~ 1g and bit 4 set in imu_ctrl.
            if bitand(obj.params.imu_ctrl, 4) ~= 0 && ~isempty(imu)
                obj.ekf.fuseGravity(imu);
            end

            % Pull output predictor toward corrected EKF state.
            if ~isempty(imu) && isfield(imu, 't')
                dt_imu = max(imu.delta_ang_dt, 1e-6);
                obj.output_pred.correctTo(obj.ekf, dt_imu);
            end
        end

        function s = stateOut(obj)
            % Controller-facing state — output predictor's corrected state,
            % with gyro reading approximated from latest IMU minus EKF bias.
            imu = obj.sensors.vehicleImu();
            omega = [];
            if ~isempty(imu) && isfield(imu, 'gyro_b'), omega = imu.gyro_b; end
            s = obj.output_pred.stateOut(omega);
        end

        function reset(obj)
            % Full reset: sensor clocks/queues, EKF, output predictor, and the
            % per-sample staleness trackers. Without resetting the sensors and
            % the last_*_t trackers, a sim-time restart leaves the IMU sample
            % frozen at its pre-reset timestamp and the EKF predict gated off.
            obj.sensors.reset();
            obj.ekf.reset();
            obj.output_pred.reset();
            obj.last_imu_t  = -inf;
            obj.last_baro_t = -inf;
            obj.last_mag_t  = -inf;
            obj.last_gps_t  = -inf;
        end
    end
end
