function [bx, by, bz] = computeFieldBase_casadi(px, py, pz, u, params)
% COMPUTEFIELDBASE_CASADI computes the magnetic field from the base
% (permanent magnets + solenoids) at evaluation points using CasADi LUTs.
%
% Uses split single-output interpolants with .map() for efficient batched
% evaluation, producing a compact CasADi expression graph.
%
% Inputs:
%   px, py, pz - Evaluation point coordinates (1 x N, CasADi MX)
%   u          - Solenoid currents (n_sol x 1, CasADi MX)
%   params     - Parameter struct with params.luts field
%
% Outputs:
%   bx, by, bz - Magnetic field components (1 x N, CasADi MX)

    import casadi.*

    N = size(px, 2);
    n_sol = length(params.solenoids.r);
    eps_val = 1e-9;

    % Create mapped versions for this N
    perm_bx_map = params.luts.perm_bx.map(N);
    perm_by_map = params.luts.perm_by.map(N);
    perm_bz_map = params.luts.perm_bz.map(N);
    sol_brho_map = params.luts.sol_brho.map(N);
    sol_bz_map   = params.luts.sol_bz.map(N);

    %% Permanent magnets - batched 3D LUT via .map()
    p_input = [px; py; pz];   % 3 x N
    bx = perm_bx_map(p_input); % 1 x N
    by = perm_by_map(p_input); % 1 x N
    bz = perm_bz_map(p_input); % 1 x N

    %% Solenoids - batched 2D LUT via .map()
    for i = 1:n_sol
        dx = px - params.solenoids.x(i);
        dy = py - params.solenoids.y(i);
        dz = pz - params.solenoids.z(i);
        rho = sqrt(dx.^2 + dy.^2);

        sol_input = [dz; rho];              % 2 x N
        brho = sol_brho_map(sol_input);     % 1 x N
        bz_f = sol_bz_map(sol_input);       % 1 x N

        % Polar -> Cartesian, scale by current
        bx = bx + brho .* (dx ./ (rho + eps_val)) * u(i);
        by = by + brho .* (dy ./ (rho + eps_val)) * u(i);
        bz = bz + bz_f * u(i);
    end
end
