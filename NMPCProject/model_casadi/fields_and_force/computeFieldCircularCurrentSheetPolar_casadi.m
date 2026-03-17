function [brho, bz] = computeFieldCircularCurrentSheetPolar_casadi(rho, z, r, l, I, mu0)
% COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR_CASADI  Magnetic field of a cylindrical
%   current sheet (ideal solenoid), CasADi-compatible.
%   Uses ellipke_casadi + ellipP_casadi (no LUTs).
%
%   The current sheet is centered at the origin, axis along z, radius r, length l.
%
%   Inputs:
%     rho  - radial distance (1 x N, CasADi MX or double)
%     z    - axial distance (1 x N, CasADi MX or double)
%     r    - sheet radius (scalar, double)
%     l    - sheet length (scalar, double)
%     I    - current (scalar, double or CasADi MX)
%     mu0  - permeability (scalar, double)
%
%   Outputs:
%     brho - radial field component (1 x N)
%     bz   - axial field component (1 x N)

    import casadi.*

    eps_val = 1e-9;

    % Safeguard rho to avoid rho=r and rho=0 singularities
    rho_s = rho + eps_val;

    % Common parameters
    h2 = 4 * r * rho_s ./ (r + rho_s).^2;
    c  = mu0 * I ./ (4 * pi * sqrt(r .* rho_s)) / l;

    zetap = z + l/2;
    zetan = z - l/2;

    k2p = 4 * r * rho_s ./ ((r + rho_s).^2 + zetap.^2);
    k2n = 4 * r * rho_s ./ ((r + rho_s).^2 + zetan.^2);

    k2p = fmin(fmax(k2p, 0), 1 - 1e-10);
    k2n = fmin(fmax(k2n, 0), 1 - 1e-10);

    % Complete elliptic integrals K, E
    [Kp, Ep] = ellipke_casadi(k2p);
    [Kn, En] = ellipke_casadi(k2n);

    % Third-kind elliptic integral Pi
    Pp = ellipP_casadi(h2, k2p);
    Pn = ellipP_casadi(h2, k2n);

    % Radial component: brho
    brhop = c .* 2 .* r .* (k2p - 2) ./ sqrt(k2p) .* (Kp - 2 ./ (2 - k2p) .* Ep);
    brhon = c .* 2 .* r .* (k2n - 2) ./ sqrt(k2n) .* (Kn - 2 ./ (2 - k2n) .* En);
    brho = brhop - brhon;

    % Axial component: bz
    alpha = (rho_s - r) ./ (rho_s + r);

    bzp = c .* zetap .* sqrt(k2p) .* (Kp - alpha .* Pp);
    bzn = c .* zetan .* sqrt(k2n) .* (Kn - alpha .* Pn);
    bz = bzp - bzn;
end
