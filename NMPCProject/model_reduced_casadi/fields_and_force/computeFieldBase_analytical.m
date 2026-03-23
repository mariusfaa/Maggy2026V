%% --- Analytical field computation using CasADi Function + map ---
%
% Split into two specialized functions with baked-in constants:
%
%   f_perm(px, py, pz)     — 3 inputs, Jacobian 3×3, structurally zero w.r.t. u
%   f_sol(px, py, pz, I)   — 4 inputs, Jacobian 3×4, geometry baked in
%
% Constants (source positions, radii, lengths) are substituted numerically
% into the SX graph at model creation time, so CasADi can:
%   - Fold constant subexpressions in the generated C code
%   - Produce smaller Jacobian blocks (3×3 and 3×4 instead of 3×9)
%   - Know that df_perm/du = 0 structurally (no u in the expression)
%
function [bx, by, bz] = computeFieldBase_analytical(px, py, pz, u, params, modelId)
    import casadi.*

    eps_val = 1e-9;
    mu0 = params.physical.mu0;
    n_perm = length(params.permanent.r);
    n_sol  = length(params.solenoids.r);
    nEval  = size(px, 2);

    % =====================================================================
    % Permanent magnets: f_perm(px, py, pz) — all source params baked in
    % =====================================================================
    I_perm_vals = params.permanent.J / mu0 * params.permanent.l;

    bx_acc = zeros(1, nEval);  % numeric accumulator (will become MX via addition)
    by_acc = zeros(1, nEval);
    bz_acc = zeros(1, nEval);

    for i = 1:n_perm
        % Build SX function with this source's constants baked in
        px_s = SX.sym('px');
        py_s = SX.sym('py');
        pz_s = SX.sym('pz');

        dx_s  = px_s - params.permanent.x(i);
        dy_s  = py_s - params.permanent.y(i);
        dz_s  = pz_s - params.permanent.z(i);
        rho_s = sqrt(dx_s^2 + dy_s^2);

        switch modelId
            case MaglevModel.Fast
                [brho_s, bz_s] = computeFieldCircularWirePolar_casadi(...
                    rho_s, dz_s, params.permanent.r(i), I_perm_vals(i), mu0);
            otherwise
                [brho_s, bz_s] = computeFieldCircularCurrentSheetPolar_casadi(...
                    rho_s, dz_s, params.permanent.r(i), params.permanent.l(i), I_perm_vals(i), mu0);
        end

        bx_s = brho_s * dx_s / (rho_s + eps_val);
        by_s = brho_s * dy_s / (rho_s + eps_val);

        f_p = Function(sprintf('f_perm%d', i), ...
            {px_s, py_s, pz_s}, {bx_s, by_s, bz_s});
        f_p_map = f_p.map(nEval);

        [bx_i, by_i, bz_i] = f_p_map(px, py, pz);
        bx_acc = bx_acc + bx_i;
        by_acc = by_acc + by_i;
        bz_acc = bz_acc + bz_i;
    end

    bx = bx_acc;
    by = by_acc;
    bz = bz_acc;

    % =====================================================================
    % Solenoids: f_sol(px, py, pz, I) — geometry baked in, current symbolic
    % =====================================================================
    for i = 1:n_sol
        px_s = SX.sym('px');
        py_s = SX.sym('py');
        pz_s = SX.sym('pz');
        I_s  = SX.sym('I');

        dx_s  = px_s - params.solenoids.x(i);
        dy_s  = py_s - params.solenoids.y(i);
        dz_s  = pz_s - params.solenoids.z(i);
        rho_s = sqrt(dx_s^2 + dy_s^2);

        switch modelId
            case MaglevModel.Fast
                [brho_s, bz_s] = computeFieldCircularWirePolar_casadi(...
                    rho_s, dz_s, params.solenoids.r(i), I_s, mu0);
            otherwise
                [brho_s, bz_s] = computeFieldCircularCurrentSheetPolar_casadi(...
                    rho_s, dz_s, params.solenoids.r(i), params.solenoids.l(i), I_s, mu0);
        end

        bx_s = brho_s * dx_s / (rho_s + eps_val);
        by_s = brho_s * dy_s / (rho_s + eps_val);

        f_s = Function(sprintf('f_sol%d', i), ...
            {px_s, py_s, pz_s, I_s}, {bx_s, by_s, bz_s});
        f_s_map = f_s.map(nEval);

        I_sym = params.solenoids.nw * u(i);
        I_rep = repmat(I_sym, 1, nEval);

        [bx_i, by_i, bz_i] = f_s_map(px, py, pz, I_rep);
        bx = bx + bx_i;
        by = by + by_i;
        bz = bz + bz_i;
    end
end
