function test_sensor_voter()
% Unit tests for SensorHub / VotedSensors / DataValidator.
% Verifies priority-based selection and switchover-on-fault behaviour
% per src/lib/validation/DataValidatorGroup.cpp:140-246.

addpaths();
earth = EarthModel(0, 0, 0);
gt    = stationary_state();

%% Case 1: highest-priority IMU is selected by default.
%  ICM-45686 (priority 90) is the voted primary -- it is the realistic 6X MEMS
%  IMU that feeds the EKF and the VIO bridge. The tactical ADIS16470 (now
%  priority 75) is the failover backup.
hub = SensorHub(earth);
T = 0.5;
for t = 0:1e-3:T, hub.step(t, gt); end
sel = hub.sensorSelection();
icm_id  = uint32(1) * 65536 + uint32(1) * 256 + uint32(0);   % ImuICM45686 (primary)
adis_id = uint32(3) * 65536 + uint32(3) * 256 + uint32(2);   % ImuADIS16470 (backup)
assert(sel.gyro_device_id == icm_id, ...
    sprintf('ICM-45686 (priority 90) selected, got device_id=%d', sel.gyro_device_id));

%% Case 2: forcing the primary's confidence to zero (saturated error
%  density) causes failover to next-best (the ADIS16470, priority 75).
%  Validator.put() refreshes last_t each tick, so the test pumps the error
%  counter via direct field manipulation, then steps once to re-evaluate.
v_pri = hub.imu.validators{hub.imu.selected_idx};
v_pri.error_density = DataValidator.ERROR_DENSITY_WINDOW;   % confidence -> 0
hub.imu.updateSelection(T);
sel2 = hub.sensorSelection();
assert(sel2.gyro_device_id ~= icm_id, ...
    'After saturating error density on the ICM-45686, selection should switch');
assert(sel2.gyro_device_id == adis_id, ...
    'Failover should pick the next-priority IMU (ADIS16470)');
assert(hub.imu.failover_count >= 1, 'failover_count incremented');

%% Case 3: voted IMU sample is the latest from the selected sensor.
imu_pub = hub.vehicleImu();
assert(~isempty(imu_pub) && isfield(imu_pub, 'gyro_b'), 'voted vehicle_imu sample shape');
assert(imu_pub.device_id == sel2.gyro_device_id, 'sample device_id matches selection');

%% Case 4: baro voter picks ICP-201XX (priority 75) over BMP388 (priority 50).
icp_id = uint32(10) * 65536 + uint32(4) * 256 + uint32(0);
assert(sel.baro_device_id == icp_id, 'ICP-201XX selected as primary baro');

%% Case 5: mag voter picks IST8310 (external, priority 75) over BMM150 (50).
ist_id = uint32(21) * 65536 + uint32(1) * 256 + uint32(1);
assert(sel.mag_device_id == ist_id, 'External IST8310 selected as primary mag');

fprintf('test_sensor_voter: PASS\n');
end


function gt = stationary_state()
gt.position_ned    = [0; 0; -10];
gt.velocity_ned    = [0; 0; 0];
gt.attitude_q      = [1; 0; 0; 0];
gt.angular_vel_b   = [0; 0; 0];
gt.acceleration_ned= [0; 0; 0];
end


function addpaths()
here = fileparts(mfilename('fullpath'));
root = fileparts(fileparts(here));
addpath(fullfile(root, 'src', 'math'));
addpath(fullfile(root, 'src', 'sensors'));
addpath(fullfile(root, 'src', 'sensors', 'imu'));
addpath(fullfile(root, 'src', 'sensors', 'baro'));
addpath(fullfile(root, 'src', 'sensors', 'mag'));
addpath(fullfile(root, 'src', 'sensors', 'gnss'));
addpath(fullfile(root, 'src', 'sensors', 'voter'));
end
