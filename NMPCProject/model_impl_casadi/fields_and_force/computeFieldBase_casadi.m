function [bx, by, bz] = computeFieldBase_casadi(px, py, pz, u, params, modelId)
% COMPUTEFIELDBASE_CASADI computes the magnetic field from the base
% (permanent magnets + solenoids) at evaluation points.
%
% Supports two field computation methods:
%   'lut'        - LUT interpolants with .map() (default if params.luts exists)
%   'analytical' - Direct ellipke_casadi evaluation (no LUTs, smooth AD)
%
% The method is selected via params.field_method ('lut' or 'analytical').
% modelId selects Fast (wire) vs Accurate (current sheet) for analytical mode.
%
% Inputs:
%   px, py, pz - Evaluation point coordinates (1 x N, CasADi MX)
%   u          - Solenoid currents (n_sol x 1, CasADi MX)
%   params     - Parameter struct
%   modelId    - MaglevModel enum (default: MaglevModel.Accurate)
%
% Outputs:
%   bx, by, bz - Magnetic field components (1 x N, CasADi MX)

    import casadi.*

    if nargin < 6, modelId = MaglevModel.Accurate; end

    % Choose field computation method
    if isfield(params, 'field_method') && strcmp(params.field_method, 'analytical')
        [bx, by, bz] = computeField_analytical(px, py, pz, u, params, modelId);
    else
        [bx, by, bz] = computeField_lut(px, py, pz, u, params);
    end
end

%% --- LUT-based field computation (original) ---
function [bx, by, bz] = computeField_lut(px, py, pz, u, params)
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

%% --- Analytical field computation (ellipke_casadi, no LUTs) ---
function [bx, by, bz] = computeField_analytical(px, py, pz, u, params, modelId)
    import casadi.*

    eps_val = 1e-9;
    mu0 = params.physical.mu0;
    n_perm = length(params.permanent.r);
    n_sol  = length(params.solenoids.r);

    bx = MX.zeros(1, size(px, 2));
    by = MX.zeros(1, size(px, 2));
    bz = MX.zeros(1, size(px, 2));

    %% Permanent magnets
    for i = 1:n_perm
        dx = px - params.permanent.x(i);
        dy = py - params.permanent.y(i);
        dz = pz - params.permanent.z(i);
        rho = sqrt(dx.^2 + dy.^2);

        % Equivalent current: I = J/mu0 * l
        I_perm = params.permanent.J / mu0 * params.permanent.l(i);

        switch modelId
            case MaglevModel.Fast
                [brho_p, bz_p] = computeFieldCircularWirePolar_casadi(...
                    rho, dz, params.permanent.r(i), I_perm, mu0);
            otherwise  % Accurate
                [brho_p, bz_p] = computeFieldCircularCurrentSheetPolar_casadi(...
                    rho, dz, params.permanent.r(i), params.permanent.l(i), I_perm, mu0);
        end

        % Polar to Cartesian
        bx = bx + brho_p .* (dx ./ (rho + eps_val));
        by = by + brho_p .* (dy ./ (rho + eps_val));
        bz = bz + bz_p;
    end

    %% Solenoids
    for i = 1:n_sol
        dx = px - params.solenoids.x(i);
        dy = py - params.solenoids.y(i);
        dz = pz - params.solenoids.z(i);
        rho = sqrt(dx.^2 + dy.^2);

        switch modelId
            case MaglevModel.Fast
                % Wire model, I=1 (scale by nw*u(i) after)
                [brho_s, bz_s] = computeFieldCircularWirePolar_casadi(...
                    rho, dz, params.solenoids.r(i), 1, mu0);
            otherwise  % Accurate
                % Current sheet model, I=1 (scale by nw*u(i) after)
                [brho_s, bz_s] = computeFieldCircularCurrentSheetPolar_casadi(...
                    rho, dz, params.solenoids.r(i), params.solenoids.l(i), 1, mu0);
        end

        % Scale by nw * current, polar to Cartesian
        scale = params.solenoids.nw * u(i);
        bx = bx + brho_s .* (dx ./ (rho + eps_val)) .* scale;
        by = by + brho_s .* (dy ./ (rho + eps_val)) .* scale;
        bz = bz + bz_s .* scale;
    end
end
