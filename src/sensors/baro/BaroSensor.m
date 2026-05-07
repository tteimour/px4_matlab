classdef BaroSensor < Sensor
% BaroSensor — abstract base for barometric pressure sensors.
%
% Concrete chips (ICP-201XX, BMP388) set:
%   * sample_rate_hz   — chip ODR
%   * latency_s        — driver latency
%   * alt_noise_m      — altitude noise std-dev (m)
%   * bias_walk_m_per_s_sqrt — slow drift random walk (m/sqrt(s))
%   * turn_on_bias_m   — turn-on bias 1-sigma (m)
%   * temp_drift_m_per_K — altitude error per K of temperature drift
%
% Output sample fields:
%   t              sample time (s)
%   altitude_m     measured altitude above MSL (m)
%   pressure_pa    derived pressure (Pa) using ISA standard atmosphere
%   temperature_c  sensor temperature (deg C, simple model)
%   instance, device_id
%
% Note on the conversion: PX4 publishes baro as altitude_m
% (vehicle_air_data_s.baro_alt_meter). The plant's truth is NED z; we
% convert to altitude as alt = earth.alt0_m - gt.position_ned(3).

    properties
        alt_noise_m
        bias_walk_m_per_s_sqrt
        turn_on_bias_m
        temp_drift_m_per_K
        earth                  % EarthModel handle
    end

    properties (Access = protected)
        bias_m_                % current slow bias (m)
        last_t_
    end

    methods
        function obj = BaroSensor(rate_hz, latency_s, priority, instance, ...
                                  device_id, earth)
            obj@Sensor(rate_hz, latency_s, priority, instance, device_id);
            obj.earth   = earth;
            obj.bias_m_ = 0.0;
            obj.last_t_ = 0.0;
        end

        function initBias(obj)
            obj.bias_m_ = obj.turn_on_bias_m * randn(obj.rng);
        end

        function reset(obj)
            reset@Sensor(obj);
            obj.last_t_ = 0.0;
            obj.initBias();
        end
    end

    methods (Access = protected)
        function s = measure(obj, t_sample, gt)
            dt = max(t_sample - obj.last_t_, obj.sample_period_);
            obj.last_t_ = t_sample;
            obj.bias_m_ = obj.bias_m_ + obj.bias_walk_m_per_s_sqrt * sqrt(dt) * randn(obj.rng);

            % Truth altitude (above MSL).
            alt_truth = obj.earth.alt0_m - gt.position_ned(3);

            % Simple temperature model: 25C nominal + sensor-specific drift hook.
            temp_c = 25.0;

            altitude_m = alt_truth + obj.bias_m_ + obj.alt_noise_m * randn(obj.rng);

            % Convert altitude to pressure using ISA (just for the data field —
            % EKF2 baro fusion uses altitude directly).
            T0 = 288.15; L = 0.0065; P0 = 101325.0; R = 287.05; g = obj.earth.g_mps2;
            pressure_pa = P0 * (1.0 - L * altitude_m / T0)^(g / (R * L));

            s.altitude_m    = altitude_m;
            s.pressure_pa   = pressure_pa;
            s.temperature_c = temp_c;
        end
    end
end
