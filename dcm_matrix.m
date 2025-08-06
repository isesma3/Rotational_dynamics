function matrix = dcm_matrix(phi,thetta, psi)
% dcm_matrix computes the Director Cosine Matrix of a 321 rotation
% transforamtion
% Inputs:
%   phi: roll angle [rad]
%   thetta: pith angle [rad]
%   psi: yaw angle [rad]
% Output:
%   dcm_matrix: 3x3 DCM matrix

    matrix = [cos(thetta)*cos(psi), cos(thetta)*sin(psi), -sin(thetta);
        -cos(phi)*sin(psi) + sin(phi)*sin(thetta)*cos(psi), cos(phi)*cos(psi) + sin(phi)*sin(thetta)*sin(psi), ...
        sin(phi)*cos(thetta);
        sin(phi)*sin(psi) + cos(phi)*sin(thetta)*cos(psi), -sin(phi)*cos(psi) + cos(phi)*sin(thetta)*sin(psi), ...
        cos(phi)*cos(thetta)];
end