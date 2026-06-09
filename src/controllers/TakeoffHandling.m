classdef TakeoffHandling < handle
% TakeoffHandling — smooth-takeoff state machine, port of
%   src/modules/mc_pos_control/Takeoff/Takeoff.{hpp,cpp}
%
% States (Takeoff.hpp:48-54):
%   DISARMED -> SPOOLUP (COM_SPOOLUP_TIME) -> READY_FOR_TAKEOFF ->
%   RAMPUP (MPC_TKO_RAMP_T) -> FLIGHT
%
% While the state is below RAMPUP the vehicle layer forces zero thrust,
% an empty trajectory setpoint and integrator resets
% (MulticopterPositionControl.cpp:496-531); during RAMPUP updateRamp()
% slews the upward-velocity limit from a negative initial value (the
% velocity that maps to zero thrust through the velocity P gain,
% Takeoff.cpp:42-46) up to the requested climb-rate limit.

    properties (Constant)
        DISARMED          = 1
        SPOOLUP           = 2
        READY_FOR_TAKEOFF = 3
        RAMPUP            = 4
        FLIGHT            = 5
    end

    properties
        state = 1            % TakeoffHandling.DISARMED
        spoolup_time         % s, COM_SPOOLUP_TIME (commander_params.c:851)
        ramp_time            % s, MPC_TKO_RAMP_T (multicopter_takeoff_land_params.c:46)
        spoolup_elapsed = 0  % time spent armed in SPOOLUP
        ramp_progress   = 0  % 0..1 through the thrust ramp
        ramp_vz_init    = 0  % upward-speed limit at ramp start (negative)
    end

    methods
        function obj = TakeoffHandling(p)
            obj.spoolup_time = p.com.spoolup_time;
            obj.ramp_time    = p.auto.tko_ramp_t;
            obj.reset();
        end

        function reset(obj)
            obj.state           = obj.DISARMED;
            obj.spoolup_elapsed = 0;
            obj.ramp_progress   = 0;
        end

        function generateInitialRampValue(obj, velocity_p_gain)
            % Takeoff.cpp:42-46: the ramp starts from the (negative)
            % velocity setpoint that produces exactly zero thrust through
            % the vertical velocity P gain.
            velocity_p_gain = max(velocity_p_gain, 0.01);
            obj.ramp_vz_init = -9.80665 / velocity_p_gain;
        end

        function updateTakeoffState(obj, armed, landed, want_takeoff, dt)
            % Takeoff.cpp:48-110 (hysteresis replaced by an explicit
            % spoolup timer; the sim clock is noise-free).
            if ~armed
                obj.state = obj.DISARMED;
                obj.spoolup_elapsed = 0;
                return;
            end
            switch obj.state
                case obj.DISARMED
                    obj.state = obj.SPOOLUP;
                    obj.spoolup_elapsed = 0;
                case obj.SPOOLUP
                    obj.spoolup_elapsed = obj.spoolup_elapsed + dt;
                    if obj.spoolup_elapsed >= obj.spoolup_time
                        obj.state = obj.READY_FOR_TAKEOFF;
                    end
                case obj.READY_FOR_TAKEOFF
                    if want_takeoff                          % Takeoff.cpp:72-79
                        obj.state = obj.RAMPUP;
                        obj.ramp_progress = 0;
                    end
                case obj.RAMPUP
                    if obj.ramp_progress >= 1.0
                        obj.state = obj.FLIGHT;
                    end
                case obj.FLIGHT
                    if landed
                        obj.state = obj.READY_FOR_TAKEOFF;   % Takeoff.cpp:92-94
                    end
            end
        end

        function up_limit = updateRamp(obj, dt, takeoff_desired_vz)
            % Takeoff.cpp:113-135. Returns the upward-speed limit to apply
            % this cycle (negative before/at ramp start = zero thrust).
            up_limit = takeoff_desired_vz;
            if obj.state < obj.RAMPUP
                up_limit = obj.ramp_vz_init;
                return;
            end
            if obj.state == obj.RAMPUP
                if obj.ramp_time > dt
                    obj.ramp_progress = obj.ramp_progress + dt / obj.ramp_time;
                else
                    obj.ramp_progress = 1.0;
                end
                if obj.ramp_progress < 1.0
                    up_limit = obj.ramp_vz_init ...
                             + obj.ramp_progress * (takeoff_desired_vz - obj.ramp_vz_init);
                end
            end
        end
    end
end
