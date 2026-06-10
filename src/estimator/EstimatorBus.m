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

        % Controller-facing gyro low-pass (PX4 VehicleAngularVelocity /
        % IMU_GYRO_CUTOFF). The rate loop consumes obj.omega_filt; the EKF
        % predict still uses the raw gyro. See setGyroCutoff().
        gyro_lpf            % LowPassFilter2p (3-axis)
        omega_filt   = zeros(3, 1)
        gyro_lpf_primed = false

        last_imu_t   = -inf
        last_baro_t  = -inf
        last_mag_t   = -inf
        last_gps_t   = -inf

        % External-vision (VIO) aiding. When vio_enabled, OpenVINS odometry
        % (already NED-aligned by the caller) replaces GNSS as the pos/vel
        % aid. vio_sample = struct('pos_ned',3x1,'vel_ned',3x1 or [],'t',s).
        vio_enabled  = false
        vio_sample   = []
        last_vio_t   = -inf
    end

    methods
        function obj = EstimatorBus(earth, varargin)
            obj.params      = Ekf2Params();
            obj.sensors     = SensorHub(earth, varargin{:});
            obj.ekf         = Ekf2(obj.params, earth);
            obj.output_pred = OutputPredictor(obj.params, earth);
            % Gyro low-pass at the IMU rate (1 kHz). Cutoff from params
            % (IMU_GYRO_CUTOFF, default 40 Hz). 0 -> passthrough.
            obj.gyro_lpf = LowPassFilter2p(1000.0, obj.params.gyro_cutoff_hz);
        end

        function setGyroCutoff(obj, hz)
            % Reconfigure the controller-facing gyro low-pass (Hz). 0 disables
            % it (raw bias-corrected gyro to the rate loop, the pre-filter
            % behaviour). Re-primes on the next IMU sample.
            obj.gyro_lpf.setCutoff(1000.0, hz);
            obj.gyro_lpf_primed = false;
        end

        function step(obj, t, ground_truth)
            obj.sensors.step(t, ground_truth);

            % The strapdown output predictor runs at the IMU rate; the EKF
            % itself (predict + fusions + output correction) runs once per
            % downsampled EKF2_PREDICT_US window, like PX4 (ekf.cpp update()
            % runs when the downsampler hands over an accumulated sample).
            imu = obj.sensors.vehicleImu();
            new_imu = false; ekf_updated = false;
            if ~isempty(imu) && isfield(imu, 't') && imu.t > obj.last_imu_t
                % Until the filter initialises (Ekf::initialiseFilter),
                % feed it the latest baro altitude for the height seed and
                % hold the output predictor; align the output predictor to
                % the EKF state at the moment of initialisation
                % (alignOutputFilter, ekf.cpp:208).
                was_init = obj.ekf.filter_init;
                if ~was_init
                    air0 = obj.sensors.vehicleAirData();
                    if ~isempty(air0) && isfield(air0, 'altitude_m')
                        obj.ekf.setInitBaro(air0.altitude_m);
                    end
                end
                ekf_updated = obj.ekf.predict(imu);
                if obj.ekf.filter_init
                    if ~was_init
                        obj.output_pred.alignTo(obj.ekf);
                    end
                    obj.output_pred.update(imu);
                end
                obj.last_imu_t = imu.t;
                new_imu = true;
            end

            if ekf_updated
                % Fusion order matters: height first, then position/vel, then mag.
                air = obj.sensors.vehicleAirData();
                if ~isempty(air) && isfield(air, 't') && air.t > obj.last_baro_t
                    obj.ekf.fuseBaro(air);
                    obj.last_baro_t = air.t;
                end

                if obj.vio_enabled
                    % VIO replaces GNSS as the horizontal position/velocity aid.
                    % Baro (height) and mag (heading) keep fusing as normal below.
                    vs = obj.vio_sample;
                    if ~isempty(vs) && isfield(vs, 't') && vs.t > obj.last_vio_t
                        obj.ekf.fuseVioPos(vs.pos_ned);
                        if isfield(vs, 'vel_ned') && ~isempty(vs.vel_ned) ...
                                && bitand(obj.params.gps_ctrl, 4) ~= 0
                            obj.ekf.fuseVioVel(vs.vel_ned);
                        end
                        obj.last_vio_t = vs.t;
                    end
                else
                    gps = obj.sensors.vehicleGpsPosition();
                    if ~isempty(gps) && isfield(gps, 't') && gps.t > obj.last_gps_t ...
                            && bitand(obj.params.gps_ctrl, 1) ~= 0
                        obj.ekf.fuseGnssPos(gps);
                        if bitand(obj.params.gps_ctrl, 4) ~= 0
                            obj.ekf.fuseGnssVel(gps);
                        end
                        obj.last_gps_t = gps.t;
                    end
                end

                % Mag fusion mode (mag_control.cpp:186-192): AUTO fuses the
                % full mag vector but only lets it update tilt after the
                % in-flight alignment (mag_3D); before that the tilt gains
                % are zeroed (heading-style). 1 = legacy heading fusion,
                % 2 = unconditional 3D, 3 = none.
                mag = obj.sensors.vehicleMagnetometer();
                if ~isempty(mag) && isfield(mag, 't') && mag.t > obj.last_mag_t
                    switch obj.params.mag_type
                        case 1
                            obj.ekf.fuseMagHeading(mag);
                        case 2
                            obj.ekf.fuseMag3D(mag, true);
                        case 3
                            % none
                        otherwise   % 0 = AUTO
                            obj.ekf.fuseMag3D(mag, obj.ekf.mag_aligned_in_flight);
                    end
                    obj.last_mag_t = mag.t;
                end

                % Gravity fusion if bit 4 set in imu_ctrl (gating inside:
                % accel ~1g or at rest, and no horizontal aiding active).
                if bitand(obj.params.imu_ctrl, 4) ~= 0
                    obj.ekf.fuseGravity();
                end

                % Pull output predictor toward the corrected EKF state once
                % per EKF update (PX4 correctOutputStates).
                obj.output_pred.correctTo(obj.ekf, obj.ekf.dt_ekf);
            end

            % Controller-facing angular velocity: bias-corrected gyro (latest
            % EKF bias) through the gyro low-pass (PX4 VehicleAngularVelocity /
            % IMU_GYRO_CUTOFF). Advance once per NEW IMU sample, at the IMU rate.
            if new_imu && isfield(imu, 'gyro_b')
                omega_corr = imu.gyro_b - obj.ekf.gyro_b;
                if ~obj.gyro_lpf_primed
                    obj.gyro_lpf.reset(omega_corr);
                    obj.gyro_lpf_primed = true;
                    obj.omega_filt = omega_corr;
                else
                    obj.omega_filt = obj.gyro_lpf.apply(omega_corr);
                end
            end
        end

        function setVio(obj, sample)
            % Push the latest NED-aligned VIO measurement (consumed by step()
            % on the next IMU tick while vio_enabled). sample fields:
            % pos_ned (3x1), vel_ned (3x1 or []), t (sim seconds).
            obj.vio_sample = sample;
        end

        function enableVio(obj, tf)
            % Switch the pos/vel aiding source: true = VIO, false = GNSS.
            obj.vio_enabled = logical(tf);
            obj.ekf.ev_active = obj.vio_enabled;   % horizontal-aiding flag
            if ~obj.vio_enabled
                obj.vio_sample = [];
                obj.last_vio_t = -inf;
            end
            % Re-arm GNSS staleness so it resumes cleanly when VIO turns off.
            obj.last_gps_t = -inf;
        end

        function setInAir(obj, tf)
            % Flight-phase flag from the vehicle layer (PX4 commander/land
            % detector -> ekf2 control flags). Gates mag-3D tilt updates
            % and the gravity-fusion at-rest bypass.
            obj.ekf.setFlightPhase(tf);
        end

        function s = stateOut(obj)
            % Controller-facing state — output predictor's corrected pos/vel/att,
            % plus the low-pass-filtered, bias-corrected angular rate (updated
            % once per IMU sample in step(), PX4 VehicleAngularVelocity-style).
            % Pure read: does NOT advance the filter, so extra stateOut() calls
            % (e.g. VIO anchoring) are side-effect free.
            s = obj.output_pred.stateOut([]);
            s.angular_vel_b = obj.omega_filt;
        end

        function reset(obj)
            % Full reset: sensor clocks/queues, EKF, output predictor, and the
            % per-sample staleness trackers. Without resetting the sensors and
            % the last_*_t trackers, a sim-time restart leaves the IMU sample
            % frozen at its pre-reset timestamp and the EKF predict gated off.
            obj.sensors.reset();
            obj.ekf.reset();
            obj.output_pred.reset();
            obj.gyro_lpf.reset([0; 0; 0]);
            obj.omega_filt      = zeros(3, 1);
            obj.gyro_lpf_primed = false;
            obj.last_imu_t  = -inf;
            obj.last_baro_t = -inf;
            obj.last_mag_t  = -inf;
            obj.last_gps_t  = -inf;
            obj.vio_enabled = false;
            obj.vio_sample  = [];
            obj.last_vio_t  = -inf;
        end
    end
end
