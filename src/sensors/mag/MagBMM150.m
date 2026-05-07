classdef MagBMM150 < MagSensor
% MagBMM150 — Bosch BMM150 magnetometer (V6X_6 internal, I2C bus 4).
%
% PX4 driver: src/drivers/magnetometer/bosch/bmm150/BMM150.{hpp,cpp}
%   ODR   = 20 Hz                (BMM150.hpp:131)
%   Range = ±1300 µT (XY) / ±2500 µT (Z) datasheet
%   Sensitivity: 0.01 µT/LSB     (BMM150.cpp:463)
%   High-accuracy preset (REPXY/REPZ HA — BMM150.hpp:132-133)
%
% Datasheet (Bosch BMM150 v1.5, HA preset):
%   RMS noise (HA mode):  0.6 µT   -> 6e-3 G  (per axis)
%   Hard-iron offset:     ±50 µT   -> 0.5 G   (board-dependent)
%   Soft-iron error:      ±5 %     scale factor

    methods
        function obj = MagBMM150(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 50; end
            if nargin < 2 || isempty(instance), instance = 0;  end

            rate_hz   = 20;
            latency_s = 30e-3;
            device_id = uint32(20) * 65536 + uint32(4) * 256 + uint32(instance);

            obj@MagSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.mag_noise_g    = 6e-3;
            obj.mag_quant_g    = 1e-4;       % 0.01 µT/LSB
            obj.mag_fs_g       = 13.0;       % ±1300 µT
            obj.hard_iron_g    = [0.3; 0.3; 0.3];
            obj.soft_iron_pct  = 0.02;
            obj.R_chip_to_body = px4_rotation(0);   % -R 0
            obj.initBias();
        end
    end
end
