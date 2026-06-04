function [pos_enu, q_enu_flu] = ned_frd_to_enu_flu(pos_ned, q_ned_frd)
%NED_FRD_TO_ENU_FLU  Convert a PX4 NED/FRD pose to a Gazebo ENU/FLU pose.
%
% px4_matlab carries vehicle state in the PX4 convention:
%   * world frame  NED  (x=North, y=East,  z=Down)
%   * body frame   FRD  (x=Forward, y=Right, z=Down)
%   * attitude     q_ned_frd = [w;x;y;z], body(FRD)->NED  (v_ned = R(q) v_frd)
%
% The PX4 SITL gz_x500 model publishes its pose on /world/default/pose/info
% in the Gazebo/ROS convention, which the Cesium-Unity scene consumes:
%   * world frame  ENU  (x=East, y=North, z=Up)
%   * body frame   FLU  (x=Forward, y=Left, z=Up)
%   * orientation  q_enu_flu = [w;x;y;z], body(FLU)->ENU
% (see cesium-unity-samples Assets/SynapSim/Components/Vehicle.cs:68-76 and
%  PoseSubscriber.cs:21, which treat the incoming pose as Gazebo ENU/FLU
%  before mapping it into Unity's left-handed frame).
%
% Derivation (two coordinate-swap matrices, both proper rotations det=+1):
%   v_enu = M v_ned,  M = [0 1 0; 1 0 0; 0 0 -1]    (NED->ENU)
%   v_flu = B v_frd,  B = [1 0 0; 0 -1 0; 0 0 -1]    (FRD->FLU, 180 deg about fwd)
% A vector known in body(FLU) maps to ENU as
%   v_enu = M * R_ned_frd * B' * v_flu      =>   R_enu_flu = M * R_ned_frd * B'
% (B' = B and M' = M; both are symmetric involutions).
%
% Verified against hand-computed cases in tests/unit/test_frame_bridge.m.
%
% Inputs / outputs are column vectors; quaternions are [w;x;y;z].

M = [0 1 0; 1 0 0; 0 0 -1];
B = [1 0 0; 0 -1 0; 0 0 -1];

pos_enu = M * pos_ned(:);

R_ned_frd = quat_to_dcm(q_ned_frd);     % body(FRD) -> NED
R_enu_flu = M * R_ned_frd * B.';        % body(FLU) -> ENU
q_enu_flu = dcm_to_quat(R_enu_flu);     % [w;x;y;z], normalized
end
