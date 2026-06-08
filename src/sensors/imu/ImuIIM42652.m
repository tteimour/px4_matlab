classdef ImuIIM42652 < ImuSensor
% ImuIIM42652 — Invensense IIM-42652 industrial 6-DOF IMU on V6X_6 SPI2.
%
% PX4 driver:  src/drivers/imu/invensense/iim42652/IIM42652.{hpp,cpp}
%   ODR config:        IIM42652.hpp:73-75 (8000 Hz FIFO)
%   Gyro FS ±2000 dps: IIM42652.hpp:189   (GYRO_FS_SEL = 2000_DPS)
%   Accel FS ±16 g:    IIM42652.hpp:190   (ACCEL_FS_SEL = 16G)
%   Accel scale 1/8192 g/LSB: IIM42652.cpp:698
%   Gyro scale  1/131  dps/LSB: IIM42652.cpp:786, 796
%   AAF: 585 Hz       IIM42652.hpp:208-220
%
% Datasheet (TDK IIM-42652 rev 1.4) — industrial-grade vs ICM-45686:
%   Gyro  noise density:  2.8 mdps/√Hz       -> 4.89e-5 rad/s/√Hz
%   Accel noise density:  70  µg/√Hz         -> 6.87e-4 m/s²/√Hz
%   Gyro  bias instability: 2.5 °/h         -> 1.21e-5 rad/s
%   Accel bias instability: 25 µg            -> 2.45e-4 m/s²
%   Gyro  turn-on bias:    ±0.25 dps        -> 4.4e-3 rad/s
%   Accel turn-on bias:    ±20 mg           -> 0.196 m/s²
%
% Sim publishes at 1 kHz (post-FIFO-integrator); native FIFO is 8 kHz.

    methods
        function obj = ImuIIM42652(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 60; end
            if nargin < 2 || isempty(instance), instance = 1;  end

            rate_hz   = 1000;
            latency_s = 1.5e-3;
            device_id = uint32(2) * 65536 + uint32(2) * 256 + uint32(instance);

            obj@ImuSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.gyro_fs_dps    = 2000;
            obj.accel_fs_g     = 16;
            obj.gyro_quant_dps = 1/131;
            obj.accel_quant_g  = 1/8192;

            obj.gyro_nd        = 4.89e-5;
            obj.accel_nd       = 6.87e-4;
            obj.gyro_brw       = 1.21e-5;
            obj.accel_brw      = 2.45e-4;
            obj.gyro_turn_on   = 4.4e-3;
            obj.accel_turn_on  = 0.196;

            % Slightly noisier vibration response than the SPI1 IMU
            % because the SPI2 mount on V6X_6 is closer to the frame.
            obj.gyro_vib_gain  = 0.0;   % off by default (was 0.18)
            obj.accel_vib_gain = 0.0;   % off by default (was 4.0 -- starved VIO scale)

            obj.R_chip_to_body = px4_rotation(6);   % -R 6 (yaw 270)
            obj.initBias();
        end
    end
end
