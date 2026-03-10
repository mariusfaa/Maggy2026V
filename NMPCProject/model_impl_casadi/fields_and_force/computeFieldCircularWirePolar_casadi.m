function [brho, bz] = computeFieldCircularWirePolar_casadi(rho, z, r, I, mu0)
% COMPUTEFIELDCIRCULARWIREPOLAR_CASADI  Magnetic field of a circular wire loop.
%   CasADi-compatible version using ellipke_casadi (Hastings approximation).
%   No LUTs — direct analytical evaluation with smooth, AD-friendly expressions.
%
%   The wire is centered at the origin, lies in the xy-plane, has radius r,
%   and carries current I.
%
%   Inputs:
%     rho  - radial distance from wire axis (1 x N, CasADi MX or double)
%     z    - axial distance from wire plane (1 x N, CasADi MX or double)
%     r    - wire loop radius (scalar, double)
%     I    - current (scalar, double or CasADi MX)
%     mu0  - permeability of free space (scalar, double)
%
%   Outputs:
%     brho - radial field component (1 x N)
%     bz   - axial field component (1 x N)

    import casadi.*

    eps_val = 1e-9;

    % Safeguard rho to avoid division by zero
    rho_s = rho + eps_val;

    % Common factor
    c = mu0 * I ./ (4 * pi * sqrt(r .* rho_s));

    % Elliptic integral parameter k^2
    k2 = 4 * r * rho_s ./ ((r + rho_s).^2 + z.^2);
    k2 = fmin(fmax(k2, 0), 1 - 1e-6);

    % Complete elliptic integrals (Hastings polynomial + log approximation)
    [K, E] = ellipke_casadi(k2);
    %[K, E] = ellipke_approx(k2);
    %[K,E] = ellipke_agm(k2);

    % Denominator for field expressions (avoid division by zero)
    denom = (rho_s - r).^2 + z.^2 + eps_val;

    % Radial component
    brho = -(z ./ rho_s) .* c .* sqrt(k2) .* ...
           (K - (rho_s.^2 + r^2 + z.^2) ./ denom .* E);

    % Axial component
    bz = c .* sqrt(k2) .* ...
         (K - (rho_s.^2 - r^2 + z.^2) ./ denom .* E);
end
