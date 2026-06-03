function R = computeRotationMatrix(alpha, beta, gamma)
% COMPUTEROTATIONMATRIX  ZYX Euler rotation matrix for CasADi symbolics.
%
%   R = computeRotationMatrix(alpha, beta, gamma)
%
%   Computes R = Rz(gamma) * Ry(beta) * Rx(alpha), the standard aerospace
%   ZYX (yaw-pitch-roll) convention.
%
% Inputs:
%   alpha  - Roll  angle about x-axis (CasADi SX/MX or double)
%   beta   - Pitch angle about y-axis
%   gamma  - Yaw   angle about z-axis
%
% Output:
%   R      - 3x3 rotation matrix (CasADi SX/MX or double)

    import casadi.*

    Rx = [1, 0,           0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha),  cos(alpha)];

    Ry = [ cos(beta), 0, sin(beta);
           0,         1, 0;
          -sin(beta), 0, cos(beta)];

    Rz = [cos(gamma), -sin(gamma), 0;
          sin(gamma),  cos(gamma), 0;
          0,           0,          1];

    R = mtimes(mtimes(Rz, Ry), Rx);
end
