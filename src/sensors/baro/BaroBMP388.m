classdef BaroBMP388 < BaroSensor
% BaroBMP388 — Bosch BMP388 barometer (V6X_6 external, I2C 0x77).
%
% PX4 driver: src/drivers/barometer/bmp388/  (bmp388.h / bmp388.cpp)
%   Default ODR: 50 Hz   (bmp388.h:78, bmp388.h:330)
%
% Datasheet (Bosch BMP388 v1.6, default OSR P=4x / T=1x):
%   Pressure RMS noise:  ~3 Pa     -> ~0.25 m altitude noise
%   Long-term drift:     ±100 Pa/yr -> negligible per-flight
%   Turn-on offset:      ±50 Pa     -> ~0.4 m
%
% PX4 EKF2_BARO_NOISE = 2.0 m default — process variance includes
% atmospheric pressure variation.

    methods
        function obj = BaroBMP388(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 50; end
            if nargin < 2 || isempty(instance), instance = 1;  end

            rate_hz   = 50;
            latency_s = 25e-3;
            device_id = uint32(11) * 65536 + uint32(5) * 256 + uint32(instance);

            obj@BaroSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.alt_noise_m              = 0.25;
            obj.bias_walk_m_per_s_sqrt   = 0.01;
            obj.turn_on_bias_m           = 0.4;
            obj.temp_drift_m_per_K       = 0.1;

            obj.initBias();
        end
    end
end
