function R = px4_rotation(code)
% px4_rotation — chip-mount rotation matrix (chip frame -> FRD body)
%   for a given PX4 ROTATION_* code passed to drivers via the -R flag.
%
% Reference: src/lib/conversion/rotation.h enumerates the codes; the
% concrete matrices come from src/lib/conversion/rotation.cpp.
% This MATLAB version covers only the codes actually used by V6X_6
% sensors (extend as needed):
%   0  ROTATION_NONE                identity
%   6  ROTATION_YAW_270             yaw +270°
%   10 ROTATION_ROLL_180_YAW_90     roll 180°, then yaw 90°
%
% Frames: chip-axes -> body-FRD. Multiply chip vector by R to express
% in body. Equivalent of get_rot_matrix() in PX4.

    switch code
        case 0
            R = eye(3);
        case 6
            R = rotZ(deg2rad(270));
        case 10
            R = rotZ(deg2rad(90)) * rotX(deg2rad(180));
        otherwise
            error('px4_rotation: rotation code %d not implemented', code);
    end
end

function R = rotX(a)
    R = [1, 0, 0; 0, cos(a), -sin(a); 0, sin(a), cos(a)];
end

function R = rotZ(a)
    R = [cos(a), -sin(a), 0; sin(a), cos(a), 0; 0, 0, 1];
end
