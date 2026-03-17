%% --- Analytical field computation (ellipke_casadi, no LUTs) ---
function [bx, by, bz] = computeFieldBase_analytical(px, py, pz, u, params, modelId)
    import casadi.*

    eps_val = 1e-9;
    mu0 = params.physical.mu0;
    n_perm = length(params.permanent.r);
    n_sol  = length(params.solenoids.r);

    % Initialize with type-compatible zeros (works for both SX and MX)
    bx = px * 0;
    by = py * 0;
    bz = pz * 0;

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
                [brho_s, bz_s] = computeFieldCircularWirePolar_casadi(...
                    rho, dz, params.solenoids.r(i), 1, mu0);
            otherwise  % Accurate
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
