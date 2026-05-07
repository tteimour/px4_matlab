classdef GnssSensor < Sensor
% GnssSensor — abstract base for GNSS receivers.
%
% Concrete subclass (GnssM9N) sets:
%   * sample_rate_hz    — receiver fix rate
%   * latency_s         — pipeline latency (M9N ~110 ms per EKF2_GPS_DELAY)
%   * pos_h_noise_m     — horizontal position 1-sigma noise (m)
%   * pos_v_noise_m     — vertical position 1-sigma noise (m)
%   * vel_h_noise_mps   — horizontal velocity 1-sigma noise (m/s)
%   * vel_v_noise_mps   — vertical velocity 1-sigma noise (m/s)
%   * eph_m, epv_m      — reported accuracy estimates (m)
%   * sacc_mps          — reported speed accuracy (m/s)
%   * fix_type          — typical fix type (3 = 3D fix, 4 = DGPS, 6 = RTK fixed)
%   * sat_count         — typical satellite count
%
% Output sample fields:
%   t                sample time (s)
%   lat_deg          latitude (deg)
%   lon_deg          longitude (deg)
%   alt_m            altitude above MSL (m)
%   pos_ned          [3x1] convenience NED position relative to origin (m)
%   vel_ned          [3x1] NED velocity (m/s)
%   eph, epv         (m) reported horizontal/vertical accuracy
%   s_variance_mps2  variance reported on speed (m/s)^2
%   fix_type, satellites_used
%   instance, device_id
%
% Reference: EKF2_GPS_* defaults in src/modules/ekf2/params_gnss.yaml
%   EKF2_GPS_P_NOISE = 0.5 m, EKF2_GPS_V_NOISE = 0.3 m/s,
%   EKF2_GPS_DELAY = 110 ms, EKF2_REQ_NSATS = 6,
%   EKF2_REQ_EPH = 3.0 m, EKF2_REQ_EPV = 5.0 m, EKF2_REQ_SACC = 0.5 m/s.

    properties
        pos_h_noise_m
        pos_v_noise_m
        vel_h_noise_mps
        vel_v_noise_mps
        eph_m
        epv_m
        sacc_mps
        fix_type
        sat_count
        earth
        dropout_active   % logical override for fault injection
    end

    methods
        function obj = GnssSensor(rate_hz, latency_s, priority, instance, ...
                                  device_id, earth)
            obj@Sensor(rate_hz, latency_s, priority, instance, device_id);
            obj.earth = earth;
            obj.dropout_active = false;
        end

        function setDropout(obj, on)
            obj.dropout_active = logical(on);
        end
    end

    methods (Access = protected)
        function s = measure(obj, ~, gt)
            n = randn(obj.rng, 3, 1);
            v = randn(obj.rng, 3, 1);

            pos_ned = gt.position_ned + ...
                [obj.pos_h_noise_m; obj.pos_h_noise_m; obj.pos_v_noise_m] .* n;
            vel_ned = gt.velocity_ned + ...
                [obj.vel_h_noise_mps; obj.vel_h_noise_mps; obj.vel_v_noise_mps] .* v;

            [lat, lon, alt] = obj.earth.nedToLla(pos_ned);

            s.lat_deg         = lat;
            s.lon_deg         = lon;
            s.alt_m           = alt;
            s.pos_ned         = pos_ned;
            s.vel_ned         = vel_ned;
            s.eph             = obj.eph_m;
            s.epv             = obj.epv_m;
            s.s_variance_mps2 = obj.sacc_mps^2;
            s.fix_type        = obj.fix_type;
            s.satellites_used = obj.sat_count;

            if obj.dropout_active
                s.fix_type        = 0;
                s.satellites_used = 0;
                s.eph             = 1e3;
                s.epv             = 1e3;
            end
        end
    end
end
