function c = crossProduct3D(a, b)
% CROSSPRODUCT3D  Column-wise cross product of 3xN matrices.
%
%   c = crossProduct3D(a, b)
%
%   Each column of c is the cross product of the corresponding columns
%   of a and b.  Compatible with CasADi SX/MX types.
%
% Inputs:
%   a, b - 3xN matrices (CasADi SX/MX or double)
%
% Output:
%   c    - 3xN matrix of cross products

    c = [a(2,:) .* b(3,:) - a(3,:) .* b(2,:);
         a(3,:) .* b(1,:) - a(1,:) .* b(3,:);
         a(1,:) .* b(2,:) - a(2,:) .* b(1,:)];
end
