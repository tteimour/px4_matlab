classdef StickOverrideDetector < handle
% StickOverrideDetector  PX4 RC stick-override detection ("sticks_moving").
%
% In PX4, while armed and in an auto/offboard mode, moving the RC sticks
% beyond a threshold hands control back to the pilot by switching to
% Position mode. The trigger is stick MOVEMENT (rate of change), not
% absolute deflection, so a held deflection does not keep re-triggering.
%
% PX4 references (paths under /home/teymur/git/SynapLine/synap-px4):
%   src/modules/manual_control/ManualControl.cpp:115-122
%       sticks_moving = any axis |filtered d/dt| > 0.01*COM_RC_STICK_OV
%   src/modules/manual_control/MovingDiff.hpp
%       per-axis: new_diff = (value - last)/dt, low-passed by an AlphaFilter
%       with time constant 0.1 s
%   src/lib/mathlib/math/filter/AlphaFilter.hpp:68-77
%       alpha = dt / (tau + dt);  state += alpha*(sample - state)
%   src/modules/commander/Commander.cpp:2895-2939 (uses the flag to switch
%       to NAVIGATION_STATE_POSCTL)
%
% Channel mapping (sim stick struct -> PX4 manual_control_setpoint):
%   roll  = right_x,  pitch = right_y,  yaw = left_x,  throttle = left_y

    properties (Constant)
        TAU = 0.1               % MovingDiff AlphaFilter time constant [s]
    end

    properties
        thresh                  % 0.01 * COM_RC_STICK_OV  [norm-stick / s]
        last                    % struct of last raw stick values (NaN = none)
        state                   % struct of filtered per-axis derivatives
    end

    methods
        function obj = StickOverrideDetector(rc_stick_ov_pct)
            % rc_stick_ov_pct: COM_RC_STICK_OV in percent (default 30).
            obj.thresh = 0.01 * rc_stick_ov_pct;
            obj.reset();
        end

        function reset(obj)
            % Call on every mode entry so the first samples don't produce a
            % spurious large derivative (MovingDiff leaves diff at 0 until it
            % has a previous value).
            obj.last  = struct('roll', NaN, 'pitch', NaN, 'yaw', NaN, 'thr', NaN);
            obj.state = struct('roll', 0,   'pitch', 0,   'yaw', 0,   'thr', 0);
        end

        function moving = update(obj, sticks, dt)
            r = obj.axisDiff('roll',  sticks.right_x, dt);
            p = obj.axisDiff('pitch', sticks.right_y, dt);
            y = obj.axisDiff('yaw',   sticks.left_x,  dt);
            t = obj.axisDiff('thr',   sticks.left_y,  dt);
            moving = (abs(r) > obj.thresh) || (abs(p) > obj.thresh) || ...
                     (abs(y) > obj.thresh) || (abs(t) > obj.thresh);
        end
    end

    methods (Access = private)
        function s = axisDiff(obj, ch, value, dt)
            % One MovingDiff channel: filtered derivative of `value`.
            if dt < eps
                s = obj.state.(ch); return;
            end
            if ~isnan(obj.last.(ch))
                new_diff = (value - obj.last.(ch)) / dt;
                alpha = dt / (obj.TAU + dt);                 % AlphaFilter
                obj.state.(ch) = obj.state.(ch) + alpha * (new_diff - obj.state.(ch));
            end
            obj.last.(ch) = value;
            s = obj.state.(ch);
        end
    end
end
