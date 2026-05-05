function [q_sp, thrust_body_z] = thrust_to_attitude(thr_sp_ned, yaw_sp)
% Convert a NED thrust vector + yaw setpoint into a body-to-NED attitude
% quaternion and the body-z thrust scalar.
%
% Direct port of ControlMath::thrustToAttitude() from
%   src/modules/mc_pos_control/PositionControl/ControlMath.cpp:47-51
%
% Inputs:
%   thr_sp_ned : 3x1 thrust setpoint in NED (negative-z = up in NED).
%                Convention: a hovering quad has thr_sp_ned = [0; 0; -hover].
%   yaw_sp     : desired yaw (rad)
%
% Outputs:
%   q_sp          : 4x1 attitude quaternion [w;x;y;z]
%   thrust_body_z : scalar (negative when thrust pushes body upward)

q_sp = body_z_to_attitude(-thr_sp_ned, yaw_sp);
thrust_body_z = -norm(thr_sp_ned);
end
