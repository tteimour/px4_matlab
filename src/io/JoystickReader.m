classdef JoystickReader < handle
    % JoystickReader  Read a physical game controller (e.g. PS4 DualShock 4)
    % and map its sticks to the px4_matlab manual-control convention.
    %
    % Backed by MATLAB's vrjoystick (Simulink 3D Animation toolbox), which
    % gives a non-blocking, already-normalised read of axes/buttons on Linux,
    % Windows and macOS. This is a plain function wrapper -- no Simulink model
    % is created or run.
    %
    % Stick struct produced by read() (matches FlightModeManager.m:139-143 and
    % the on-screen joysticks in run_interactive.m):
    %   left_x  = yaw      (right positive)
    %   left_y  = throttle (up positive)
    %   right_x = roll     (right positive)
    %   right_y = pitch    (up positive  => stick forward => nose down)
    %
    % Physical sticks report Y up = NEGATIVE, so the two Y axes are negated
    % by default (sign.ly = sign.ry = -1) to match the on-screen convention.
    %
    % AXIS NUMBERS ARE HARDWARE/DRIVER DEPENDENT. The defaults below are the
    % common DualShock 4 order on Linux. CONFIRM them on your pad by running
    % sim/test_joystick.m, which shows live axis indices and the mapped
    % sticks side by side, then pass a map override if they differ.
    %
    % Usage:
    %   js = JoystickReader(1);            % 1-based device id
    %   s  = js.read();                    % -> sticks struct in [-1,1]
    %   [s, btn, ax] = js.read();          % also raw buttons (0/1) and axes
    %   js.close();
    %
    % Override the mapping if test_joystick shows different indices:
    %   js = JoystickReader(1, struct('rx',4,'ry',5));

    properties
        joy             % underlying vrjoystick object
        id              % 1-based device id
        nAxes           % number of axes reported by the device
        nButtons        % number of buttons reported by the device
        map             % struct: axis index (1-based) for lx, ly, rx, ry
        sign            % struct: +1/-1 per channel to match sim convention
        deadzone = 0.0  % centre cutoff applied here; 0 = let FlightModeManager
                        % handle it (MPC_HOLD_DZ). Raise if your pad drifts.
    end

    methods
        function obj = JoystickReader(id, mapOverride)
            if nargin < 1 || isempty(id), id = 1; end
            obj.id  = id;
            obj.joy = vrjoystick(id);          % throws if no device at id
            c = caps(obj.joy);
            obj.nAxes    = c.Axes;
            obj.nButtons = c.Buttons;

            % DualShock 4 / Linux axis order (1-based), confirmed on hardware
            % via jstest + vrjoystick: axes = [LX LY L2 RX RY R2 dpadX dpadY],
            % so the right stick is on indices 4 (RX) and 5 (RY) -- index 3 is
            % the L2 trigger. Stick-up reads negative, hence sign.ly/ry = -1.
            obj.map  = struct('lx', 1, 'ly', 2, 'rx', 4, 'ry', 5);
            obj.sign = struct('lx', 1, 'ly', -1, 'rx', 1, 'ry', -1);

            if nargin >= 2 && ~isempty(mapOverride)
                fn = fieldnames(mapOverride);
                for i = 1:numel(fn)
                    obj.map.(fn{i}) = mapOverride.(fn{i});
                end
            end
        end

        function [sticks, buttons, axes] = read(obj)
            % Non-blocking read of the current latched controller state.
            [axes, buttons] = read(obj.joy);   % @vrjoystick/read

            sticks.left_x  = obj.chan(axes, obj.map.lx, obj.sign.lx);
            sticks.left_y  = obj.chan(axes, obj.map.ly, obj.sign.ly);
            sticks.right_x = obj.chan(axes, obj.map.rx, obj.sign.rx);
            sticks.right_y = obj.chan(axes, obj.map.ry, obj.sign.ry);
        end

        function close(obj)
            if ~isempty(obj.joy)
                close(obj.joy);                % @vrjoystick/close
                obj.joy = [];
            end
        end

        function delete(obj)
            obj.close();
        end
    end

    methods (Access = private)
        function v = chan(obj, axes, idx, sgn)
            % One axis -> normalised, signed, deadzoned channel in [-1,1].
            if idx < 1 || idx > numel(axes)
                v = 0; return;                 % axis not present on this pad
            end
            v = sgn * axes(idx);
            if abs(v) < obj.deadzone
                v = 0;
            end
            v = max(-1, min(1, v));
        end
    end
end
