classdef LeadCompensator < handle
%LEADCOMPENSATOR  First-order discrete lead/lag pre-filter (vectorized).
%
%   H(s) = (Ts*s + 1) / (Tp*s + 1)
%
%   Ts > Tp -> phase lead. Used as a setpoint pre-filter to advance the
%   phase of a tracking command and reduce the cascaded controller's
%   visible lag during ramps and steps. Tustin (bilinear) discretization
%   is computed each step from the runtime dt, so a fixed schedule is
%   not required.
%
%   At DC, |H| = 1, so static setpoints pass through unchanged. Lead
%   only acts on transients, and the steady-state position is unbiased.
%
%   Conventions: per-channel time constants, column-vector input.
%
%   Usage:
%       lead = LeadCompensator([0.4;0.4;0.25], [0.12;0.12;0.08]);
%       y    = lead.update(u, dt);
%       lead.reset();        % match next input on first call (no kick)
%       lead.reset(u_now);   % seed state explicitly
%
%   Note: not PX4-native. Added on top of the PX4-faithful cascade to
%   compensate the inherent ramp-tracking lag of P-only outer loops.

    properties
        Ts        % zero time constants [s], n x 1
        Tp        % pole time constants [s], n x 1
    end
    properties (Access = private)
        u_prev
        y_prev
        initialized
    end

    methods
        function obj = LeadCompensator(Ts, Tp)
            obj.Ts = Ts(:);
            obj.Tp = Tp(:);
            assert(numel(obj.Ts) == numel(obj.Tp), ...
                'LeadCompensator: Ts and Tp must have the same length');
            obj.reset();
        end

        function reset(obj, ic)
            n = numel(obj.Ts);
            if nargin >= 2 && ~isempty(ic)
                obj.u_prev = ic(:);
                obj.y_prev = ic(:);
                obj.initialized = true;
            else
                obj.u_prev = zeros(n, 1);
                obj.y_prev = zeros(n, 1);
                obj.initialized = false;
            end
        end

        function y = update(obj, u, dt)
            u = u(:);
            if ~obj.initialized
                obj.u_prev = u;
                obj.y_prev = u;
                obj.initialized = true;
                y = u;
                return;
            end
            d  = 2 * obj.Tp + dt;
            b0 = (2 * obj.Ts + dt) ./ d;
            b1 = (dt - 2 * obj.Ts) ./ d;
            a1 = (dt - 2 * obj.Tp) ./ d;
            % Per-axis NaN safety: an axis filters cleanly only if its
            % current input AND its previous input AND its previous
            % output are all finite. Otherwise pass the current input
            % through and re-seed the axis state so the filter restarts
            % from rest on the next finite sample (no kick). This lets
            % callers feed NaN sentinels for "no setpoint on this axis"
            % (e.g. PositionController's vel_sp_ff convention) without
            % poisoning the filter state.
            ok = isfinite(u) & isfinite(obj.u_prev) & isfinite(obj.y_prev);
            y = u;
            y(ok) = b0(ok) .* u(ok) + b1(ok) .* obj.u_prev(ok) ...
                  - a1(ok) .* obj.y_prev(ok);
            obj.u_prev = u;
            obj.y_prev = y;
        end
    end
end
