classdef ImuADIS16470 < ImuSensor
% ImuADIS16470 — Analog Devices ADIS-16470 tactical-grade IMU on V6X_6 SPI3.
%
% PX4 driver:  src/drivers/imu/analog_devices/adis16470/ADIS16470.{hpp,cpp}
%   SAMPLE_INTERVAL_US = 500 us  -> 2000 Hz native ODR
%     Analog_Devices_ADIS16470_registers.hpp:74
%   Gyro FS ±2000 dps          ADIS16470.cpp:383
%   Accel FS ±40 g             ADIS16470.cpp:379
%   Gyro scale  1/10 dps/LSB   ADIS16470.cpp:384  (10 LSB/°/s)
%   Accel scale 1/800 g/LSB    ADIS16470.cpp:380  (800 LSB/g)
%
% Datasheet (Analog Devices ADIS16470 rev D) — tactical grade, much
% cleaner than the Invensense parts:
%   Gyro  noise density:  0.34 mdps/√Hz      -> 5.93e-6 rad/s/√Hz
%   Accel noise density:  0.022 mg/√Hz       -> 2.16e-4 m/s²/√Hz
%   Gyro  bias instability: 8.0 °/h         -> 3.88e-5 rad/s
%   Accel bias instability: 13 µg            -> 1.27e-4 m/s²
%   Gyro  turn-on bias:    ±0.2 °/s          -> 3.5e-3 rad/s
%   Accel turn-on bias:    ±3 mg             -> 0.029 m/s²
%
% Sim publishes at 1 kHz; native is 2 kHz. Easy to bump to 2 kHz if the
% sim loop runs that fast.

    methods
        function obj = ImuADIS16470(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 90; end  % highest
            if nargin < 2 || isempty(instance), instance = 2;  end

            rate_hz   = 1000;
            latency_s = 0.8e-3;        % SPI is fast, integrator is light
            device_id = uint32(3) * 65536 + uint32(3) * 256 + uint32(instance);

            obj@ImuSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.gyro_fs_dps    = 2000;
            obj.accel_fs_g     = 40;
            obj.gyro_quant_dps = 1/10;        % 10 LSB/°/s -> 0.1 dps step
            obj.accel_quant_g  = 1/800;       % 800 LSB/g

            obj.gyro_nd        = 5.93e-6;
            obj.accel_nd       = 2.16e-4;
            obj.gyro_brw       = 3.88e-5;
            obj.accel_brw      = 1.27e-4;
            obj.gyro_turn_on   = 3.5e-3;
            obj.accel_turn_on  = 0.029;

            obj.R_chip_to_body = px4_rotation(0);   % -R 0
            obj.initBias();
        end
    end
end
