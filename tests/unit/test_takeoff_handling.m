function test_takeoff_handling()
% TakeoffHandling state machine vs hand-computed Takeoff.cpp behavior:
% DISARMED -> SPOOLUP (COM_SPOOLUP_TIME) -> READY_FOR_TAKEOFF ->
% RAMPUP (MPC_TKO_RAMP_T) -> FLIGHT, with the ramped upward-speed limit
% starting at -g/vel_p_gain (Takeoff.cpp:42-46, 113-135).

addpaths();
p  = px4_params();
tk = TakeoffHandling(p);
tk.generateInitialRampValue(4.0);          % MPC_Z_VEL_P_ACC = 4
vz0 = -9.80665 / 4.0;
assert(abs(tk.ramp_vz_init - vz0) < 1e-12, 'ramp_vz_init = -g/p');

dt = 0.02;                                  % 50 Hz position loop

% Disarmed: stays put, limit pinned at vz_init.
tk.updateTakeoffState(false, true, false, dt);
assert(tk.state == TakeoffHandling.DISARMED, 'stays DISARMED while unarmed');
assert(abs(tk.updateRamp(dt, 3.0) - vz0) < 1e-12, 'zero-thrust limit while grounded');

% Arm -> SPOOLUP for COM_SPOOLUP_TIME, then READY.
tk.updateTakeoffState(true, true, false, dt);
assert(tk.state == TakeoffHandling.SPOOLUP, 'arming enters SPOOLUP');
n_spool = ceil(p.com.spoolup_time / dt);
for i = 1:n_spool
    tk.updateTakeoffState(true, true, false, dt);
end
assert(tk.state == TakeoffHandling.READY_FOR_TAKEOFF, 'spoolup timer elapses');

% want_takeoff -> RAMPUP; ramp reaches the desired limit after ramp_time.
tk.updateTakeoffState(true, true, true, dt);
assert(tk.state == TakeoffHandling.RAMPUP, 'want_takeoff enters RAMPUP');
lim1 = tk.updateRamp(dt, 3.0);             % first ramp step
expected1 = vz0 + (dt / p.auto.tko_ramp_t) * (3.0 - vz0);
assert(abs(lim1 - expected1) < 1e-12, 'ramp limit after one step');
n_ramp = ceil(p.auto.tko_ramp_t / dt) + 1;
for i = 1:n_ramp
    lim = tk.updateRamp(dt, 3.0);
    tk.updateTakeoffState(true, false, true, dt);
end
assert(tk.state == TakeoffHandling.FLIGHT, 'ramp completion enters FLIGHT');
assert(abs(lim - 3.0) < 1e-12, 'limit equals desired climb rate in flight');

% Landing in FLIGHT returns to READY (Takeoff.cpp:92-94); disarm resets.
tk.updateTakeoffState(true, true, false, dt);
assert(tk.state == TakeoffHandling.READY_FOR_TAKEOFF, 'landing re-arms takeoff');
tk.updateTakeoffState(false, true, false, dt);
assert(tk.state == TakeoffHandling.DISARMED, 'disarm resets the machine');

fprintf('test_takeoff_handling: PASS\n');
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'params'));
addpath(fullfile(root, 'src', 'controllers'));
end
