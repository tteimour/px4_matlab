classdef EarthModel < handle
% EarthModel — gravity, magnetic field, and LLA<->NED reference for sensors.
%
% Holds the simulation's local origin (lat, lon, alt) and provides:
%   - constant gravity vector in NED
%   - WMM-style constant Earth magnetic field in NED (declination /
%     inclination / total intensity), used by mag models and EKF2 mag_I
%   - LLA <-> NED conversion for GNSS modelling
%
% No real WMM lookup — declination, inclination, intensity are
% configurable; the user can set them per simulation site.
%
% Reference for declination handling: PX4 EKF2_MAG_DECL parameter
% (`src/modules/ekf2/params_mag.yaml`) and `mag_control.cpp` declination
% fusion path.

    properties
        lat0_deg     % local origin latitude (deg)
        lon0_deg     % local origin longitude (deg)
        alt0_m       % local origin altitude above MSL (m)
        g_mps2       % gravity magnitude (m/s^2)
        mag_decl_rad % magnetic declination (rad, +E)
        mag_incl_rad % magnetic inclination (rad, +down)
        mag_field_g  % total magnetic field intensity (Gauss)
    end

    methods
        function obj = EarthModel(lat0_deg, lon0_deg, alt0_m, varargin)
            obj.lat0_deg = lat0_deg;
            obj.lon0_deg = lon0_deg;
            obj.alt0_m   = alt0_m;
            obj.g_mps2   = 9.80665;
            % Defaults roughly matching mid-latitude N. hemisphere
            % (e.g. Central Europe: decl +3 deg, incl +65 deg, |B| ~0.49 G)
            obj.mag_decl_rad = deg2rad(3.0);
            obj.mag_incl_rad = deg2rad(65.0);
            obj.mag_field_g  = 0.49;

            for k = 1:2:numel(varargin)
                switch lower(varargin{k})
                    case 'g',          obj.g_mps2       = varargin{k+1};
                    case 'decl_deg',   obj.mag_decl_rad = deg2rad(varargin{k+1});
                    case 'incl_deg',   obj.mag_incl_rad = deg2rad(varargin{k+1});
                    case 'mag_g',      obj.mag_field_g  = varargin{k+1};
                    otherwise
                        error('EarthModel: unknown option %s', varargin{k});
                end
            end
        end

        function g_ned = gravityNed(obj)
            % NED gravity: positive Z is down.
            g_ned = [0; 0; obj.g_mps2];
        end

        function B_ned = magNed(obj)
            % Earth magnetic field expressed in NED (Gauss).
            cd = cos(obj.mag_decl_rad); sd = sin(obj.mag_decl_rad);
            ci = cos(obj.mag_incl_rad); si = sin(obj.mag_incl_rad);
            B_ned = obj.mag_field_g * [ci * cd; ci * sd; si];
        end

        function [lat, lon, alt] = nedToLla(obj, ned)
            % Convert NED offset (m) from origin to (lat deg, lon deg, alt m).
            % Flat-earth approximation — sufficient for sub-100km sims.
            R_earth = 6371000.0;
            lat = obj.lat0_deg + rad2deg(ned(1) / R_earth);
            lon = obj.lon0_deg + rad2deg(ned(2) / (R_earth * cosd(obj.lat0_deg)));
            alt = obj.alt0_m  - ned(3);   % NED z is down; alt is up
        end

        function ned = llaToNed(obj, lat, lon, alt)
            % Inverse of nedToLla.
            R_earth = 6371000.0;
            ned = [ deg2rad(lat - obj.lat0_deg) * R_earth;
                    deg2rad(lon - obj.lon0_deg) * R_earth * cosd(obj.lat0_deg);
                    -(alt - obj.alt0_m) ];
        end
    end
end
