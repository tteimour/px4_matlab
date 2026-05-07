classdef GnssM9N < GnssSensor
% GnssM9N — u-blox NEO-M9N GNSS receiver (Holybro M9N puck on V6X_6).
%
% PX4 driver: src/drivers/gps/devices/src/ubx.{cpp,h}
%   M9N default measurement interval: 200 ms -> 5 Hz   (ubx.h:230)
%   "M9N can go higher than 10Hz, restricted sat count"  (ubx.cpp:567)
%   Primary message: NAV-PVT                             (ubx.cpp:462-475)
%
% PX4 EKF2 GPS defaults (params_gnss.yaml):
%   EKF2_GPS_DELAY    = 110 ms
%   EKF2_GPS_P_NOISE  = 0.5 m
%   EKF2_GPS_V_NOISE  = 0.3 m/s
%   EKF2_REQ_NSATS    = 6
%   EKF2_REQ_EPH      = 3.0 m
%   EKF2_REQ_EPV      = 5.0 m
%   EKF2_REQ_SACC     = 0.5 m/s
%
% Datasheet (u-blox NEO-M9 rev 1.5, with GPS+GLONASS+Galileo+BeiDou):
%   Horizontal CEP (open sky, after fix):  1.5 m typ
%   Velocity accuracy:                     0.05 m/s typ
%   Position update rate:                  10 Hz max (4 GNSS) / 25 Hz (single)
%
% This model fires at 10 Hz (Holybro pucks are typically configured for
% the higher rate). pos / vel noise are tuned to be a bit looser than the
% datasheet best-case to reflect mounting / multipath in the real world.

    methods
        function obj = GnssM9N(priority, instance, earth)
            if nargin < 1 || isempty(priority), priority = 75; end
            if nargin < 2 || isempty(instance), instance = 0;  end

            rate_hz   = 10;
            latency_s = 110e-3;            % matches EKF2_GPS_DELAY default
            device_id = uint32(30) * 65536 + uint32(0) * 256 + uint32(instance);

            obj@GnssSensor(rate_hz, latency_s, priority, instance, device_id, earth);

            obj.pos_h_noise_m   = 1.0;
            obj.pos_v_noise_m   = 1.5;
            obj.vel_h_noise_mps = 0.1;
            obj.vel_v_noise_mps = 0.15;
            obj.eph_m           = 1.5;
            obj.epv_m           = 2.5;
            obj.sacc_mps        = 0.1;
            obj.fix_type        = 3;        % 3D fix
            obj.sat_count       = 14;       % typical multi-constellation
        end
    end
end
