classdef MagSensor < Sensor
% MagSensor — abstract base for 3-axis magnetometers.
%
% Concrete chips (BMM150, IST8310) set:
%   * sample_rate_hz   — chip ODR
%   * latency_s        — driver latency
%   * mag_noise_g      — per-axis noise std-dev (Gauss)
%   * mag_quant_g      — quantisation step (G/LSB)
%   * mag_fs_g         — full-scale (G), for clipping
%   * hard_iron_g      — [3x1] turn-on hard-iron bias 1-sigma (G)
%   * soft_iron_pct    — soft-iron scale-factor variance (fraction)
%   * R_chip_to_body   — 3x3 rotation chip frame to FRD body
%
% Output sample fields:
%   t          sample time (s)
%   mag_b      [3x1] body-frame magnetic field (Gauss)
%   instance, device_id
%
% Earth field source: EarthModel.magNed() (constant per sim, configurable
% declination/inclination/intensity).

    properties
        mag_noise_g
        mag_quant_g
        mag_fs_g
        hard_iron_g
        soft_iron_pct
        R_chip_to_body
        earth
    end

    properties (Access = protected)
        hard_iron_           % realised hard-iron offset [3x1] G
        soft_iron_           % 3x3 soft-iron matrix
    end

    methods
        function obj = MagSensor(rate_hz, latency_s, priority, instance, ...
                                 device_id, earth)
            obj@Sensor(rate_hz, latency_s, priority, instance, device_id);
            obj.earth          = earth;
            obj.R_chip_to_body = eye(3);
            obj.hard_iron_     = zeros(3, 1);
            obj.soft_iron_     = eye(3);
        end

        function initBias(obj)
            obj.hard_iron_ = obj.hard_iron_g .* randn(obj.rng, 3, 1);
            obj.soft_iron_ = eye(3) + obj.soft_iron_pct * randn(obj.rng, 3, 3);
        end

        function reset(obj)
            reset@Sensor(obj);
            obj.initBias();
        end
    end

    methods (Access = protected)
        function s = measure(obj, ~, gt)
            B_ned   = obj.earth.magNed();
            R_b2n   = quat_to_dcm(gt.attitude_q);
            B_body  = R_b2n' * B_ned;

            % Apply hard + soft iron in chip frame.
            B_chip  = obj.R_chip_to_body' * B_body;
            B_chip  = obj.soft_iron_ * B_chip + obj.R_chip_to_body' * obj.hard_iron_;
            B_chip  = B_chip + obj.mag_noise_g * randn(obj.rng, 3, 1);
            B_chip  = obj.quantise(B_chip, obj.mag_quant_g);
            B_chip  = obj.clipFs(B_chip, obj.mag_fs_g);

            s.mag_b = obj.R_chip_to_body * B_chip;
        end

        function v = quantise(~, v, step)
            if step > 0, v = round(v / step) * step; end
        end

        function v = clipFs(~, v, fs)
            v = max(min(v, fs), -fs);
        end
    end
end
