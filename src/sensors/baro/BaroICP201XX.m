classdef BaroICP201XX < BaroSensor
% BaroICP201XX — Invensense ICP-201XX MEMS barometer (V6X_6 internal,
% I2C address 0x64).
%
% PX4 driver: src/drivers/barometer/invensense/icp201xx/ICP201XX.{hpp,cpp}
%   Default mode: OP_MODE2  (ICP201XX.hpp:98)
%     ODR  = 40 Hz, bandwidth = 10 Hz
%   FIFO readout: pressure first, then temperature  (ICP201XX.hpp:104)
%
% Datasheet (Invensense ICP-201xx rev 1.4, Mode 2):
%   Pressure noise:           ~0.5 Pa RMS  -> ~0.04 m altitude noise
%   Long-term offset drift:   ±1 hPa typ   -> ~8 m turn-on alt offset
%   Slow drift:               ~10 Pa/year  -> negligible per-flight
%
% PX4 EKF2 default expectation (params_baro.yaml):
%   EKF2_BARO_NOISE = 2.0 m  (much wider — covers chip + atmospheric)

    methods
        function obj = BaroICP201XX(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 75; end
            if nargin < 2 || isempty(instance), instance = 0;  end

            rate_hz   = 40;          % Mode 2
            latency_s = 25e-3;       % I2C + driver work-queue
            device_id = uint32(10) * 65536 + uint32(4) * 256 + uint32(instance);

            obj@BaroSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.alt_noise_m              = 0.04;
            obj.bias_walk_m_per_s_sqrt   = 0.005;
            obj.turn_on_bias_m           = 1.0;     % conservative; chip is good
            obj.temp_drift_m_per_K       = 0.05;

            obj.initBias();
        end
    end
end
