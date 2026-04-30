function [bphi, brho, bz] = computeCircularWireFieldPolarCasADi(rho, z, r, I, mu0)
% COMPUTECIRCULARWIREFIELDPOLAR  Magnetic field of a circular current loop
% in cylindrical coordinates, using complete elliptic integrals.
%
%   [bphi, brho, bz] = computeCircularWireFieldPolar(rho, z, r, I, mu0)
%
%   Implements the exact analytic solution for the B-field of a single
%   circular filament carrying current I.  Uses if_else to handle the
%   on-axis singularity (rho -> 0) in a CasADi-safe manner.
%
% Inputs:
%   rho  - Radial distance from the loop axis (1xN, CasADi SX/MX)
%   z    - Axial distance from the loop plane (1xN, CasADi SX/MX)
%   r    - Loop radius  (scalar, numeric)
%   I    - Loop current (scalar, CasADi SX/MX or numeric)
%   mu0  - Permeability of free space (scalar, numeric)
%
% Outputs:
%   bphi - Azimuthal field component  (always zero by symmetry)
%   brho - Radial field component
%   bz   - Axial field component
%
% See also: ellipke_casadi

    import casadi.*

    tol = 1e-6;

    % Guard against division by zero when rho -> 0
    rho_safe = fmax(rho, tol);

    % Pre-factor
    c = mu0 * I ./ (4 * pi * sqrt(r .* rho_safe));

    % Elliptic-integral parameter k^2
    k2 = 4 * r * rho_safe ./ ((r + rho_safe).^2 + z.^2);
    k2 = fmin(fmax(k2, 0), 1 - 1e-9);

    [K, E] = ellipke_casadi(k2);

    sqrt_k2     = sqrt(k2);
    denominator = (rho_safe - r).^2 + z.^2;

    % General off-axis expressions
    brho_general = -(z ./ rho_safe) .* c .* sqrt_k2 .* ...
                   (K - (rho_safe.^2 + r^2 + z.^2) ./ denominator .* E);
    bz_general   =  c .* sqrt_k2 .* ...
                   (K - (rho_safe.^2 - r^2 + z.^2) ./ denominator .* E);

    % Exact on-axis formula
    bz_axis = mu0 * r^2 * I ./ (2 * (r^2 + z.^2).^(3/2));

    % Branch selection (CasADi-safe)
    brho = if_else(rho < tol, 0,       brho_general);
    bz   = if_else(rho < tol, bz_axis, bz_general);

    % Azimuthal component is zero by axial symmetry
    bphi = 0 * rho;
end
