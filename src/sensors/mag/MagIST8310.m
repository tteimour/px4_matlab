classdef MagIST8310 < MagSensor
% MagIST8310 — iSentek IST8310 magnetometer (V6X_6 external, on
% Holybro GPS/compass puck, I2C bus 1).
%
% PX4 driver: src/drivers/magnetometer/isentek/ist8310/IST8310.{hpp,cpp}
%   16-bit data + 16x averaging filter (IST8310.hpp:109-110)
%   PULSE_NORMAL                       (IST8310.hpp:111)
%   Sensitivity: 1/1320 G/LSB          (IST8310.cpp:270)
%   ODR not configured in driver — chip default ~100 Hz with averaging
%
% Datasheet (iSentek IST8310 rev 1.4):
%   RMS noise (16x avg): 0.2 µT  -> 2e-3 G
%   Range:               ±16 mT  -> 1600 G  (clipping virtually never)
%   Hard-iron offset:    typically ±20 µT after board mount
%   Soft-iron error:     ±3 % typical

    methods
        function obj = MagIST8310(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 75; end  % external favoured
            if nargin < 2 || isempty(instance), instance = 1;  end

            rate_hz   = 100;
            latency_s = 15e-3;
            device_id = uint32(21) * 65536 + uint32(1) * 256 + uint32(instance);

            obj@MagSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.mag_noise_g    = 2e-3;
            obj.mag_quant_g    = 1/1320;
            obj.mag_fs_g       = 160.0;       % ±16 mT
            obj.hard_iron_g    = [0.2; 0.2; 0.2];
            obj.soft_iron_pct  = 0.015;
            obj.R_chip_to_body = px4_rotation(10);  % -R 10
            obj.initBias();
        end
    end
end
