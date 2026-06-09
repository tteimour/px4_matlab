classdef ImuICM45686 < ImuSensor
% ImuICM45686 — Invensense ICM-45686 6-DOF IMU on V6X_6 SPI1.
%
% PX4 driver:  src/drivers/imu/invensense/icm45686/ICM45686.{hpp,cpp}
%   ODR config:        ICM45686.hpp:73-75  (6400 Hz FIFO)
%   Gyro FS ±4000 dps: ICM45686.hpp:156, ICM45686.cpp:329 (GYRO_UI_FS_SEL)
%   Accel FS ±32 g:    ICM45686.hpp:157, ICM45686.cpp:328 (ACCEL_UI_FS_SEL)
%   Accel scale 1/8192 g/LSB: ICM45686.cpp:581
%   Gyro scale  1/131  dps/LSB: ICM45686.cpp:678
%   20-bit hires data, FIFO 8K, low-noise mode
%
% Datasheet (TDK ICM-45686 rev 2.0):
%   Gyro  noise density:  3.8 mdps/√Hz (typ)  -> 6.63e-5 rad/s/√Hz
%   Accel noise density:  70  µg/√Hz   (typ)  -> 6.87e-4 m/s²/√Hz
%   Gyro  bias instability: 4.5 °/h   (typ) -> 2.18e-5 rad/s
%   Accel bias instability: 100 µg    (typ) -> 9.81e-4 m/s²
%   Gyro  turn-on bias:    ±0.5 dps    (typ) -> 8.7e-3 rad/s
%   Accel turn-on bias:    ±20 mg      (typ) -> 0.196 m/s²
%
% Sim simplification: chip native FIFO is 6400 Hz; here we publish
% post-FIFO-integrator samples at 1 kHz, matching what PX4's
% vehicle_imu publishes to ekf2 in typical configurations.

    methods
        function obj = ImuICM45686(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 90; end  % voted primary
                                                                     % (realistic 6X MEMS;
                                                                     % feeds EKF + VIO bridge)
            if nargin < 2 || isempty(instance), instance = 0;  end

            rate_hz   = 1000;          % effective integrated rate
            latency_s = 1.5e-3;        % SPI + integrator + transport
            % Stable device id: chip family * 2^16 + bus * 2^8 + instance.
            device_id = uint32(1) * 65536 + uint32(1) * 256 + uint32(instance);

            obj@ImuSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.gyro_fs_dps    = 4000;
            obj.accel_fs_g     = 32;
            obj.gyro_quant_dps = 1/131;       % per driver
            obj.accel_quant_g  = 1/8192;      % per driver

            obj.gyro_nd        = 6.63e-5;     % rad/s/√Hz   (datasheet)
            obj.accel_nd       = 6.87e-4;     % m/s²/√Hz    (datasheet)
            obj.gyro_brw       = 2.18e-5;     % rad/s/√s    (bias instability)
            obj.accel_brw      = 9.81e-4;     % m/s²/√s
            obj.gyro_turn_on   = 8.7e-3;      % rad/s
            obj.accel_turn_on  = 0.196;       % m/s²

            % Motor vibration coupling — typical "well-mounted but not
            % isolated" IMU in a multicopter (cf. PX4 sensor_combined logs).
            obj.gyro_vib_gain  = 0.0;         % rad/s   per unit vib_level (off by default; was 0.20)
            obj.accel_vib_gain = 0.0;         % m/s^2   per unit vib_level (off by default; was 4.5 -- starved VIO scale)

            obj.R_chip_to_body = px4_rotation(10);  % -R 10
            obj.initBias();
        end
    end
end
