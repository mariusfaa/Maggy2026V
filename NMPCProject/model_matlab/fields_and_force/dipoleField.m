function [bx, by, bz] = dipoleField(dx, dy, dz, mx, my, mz, mu0)
% DIPOLEFIELD  Magnetic field of a point dipole.
%
%   B = (mu0/4pi) * [3*(m.r)*r/r^5 - m/r^3]
%
% Inputs:
%   dx, dy, dz - displacement from dipole to field point (arrays ok)
%   mx, my, mz - dipole moment components (scalar)
%   mu0        - permeability of free space

    eps_val = 1e-12;
    r2 = dx.^2 + dy.^2 + dz.^2 + eps_val;
    r  = sqrt(r2);
    r3 = r2 .* r;
    r5 = r2.^2 .* r;

    m_dot_r = mx*dx + my*dy + mz*dz;

    coeff = mu0 / (4 * pi);

    bx = coeff * (3 * m_dot_r .* dx ./ r5 - mx ./ r3);
    by = coeff * (3 * m_dot_r .* dy ./ r5 - my ./ r3);
    bz = coeff * (3 * m_dot_r .* dz ./ r5 - mz ./ r3);
end
