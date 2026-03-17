%% --- Analytical field computation using CasADi Function + map ---
%
% Architecture: define a SCALAR SX Function for one source at one eval
% point, then .map() it over all (n_sources × nEval) pairs. With MX
% outer symbols, the map structure is preserved in the expression graph:
%   - acados generates ONE C function body, called in a loop
%   - Jacobian is ONE block, applied N times (block-diagonal)
%   - ~N× reduction in generated code size and AD cost
%
function [bx, by, bz] = computeFieldBase_analytical(px, py, pz, u, params, modelId)
    import casadi.*

    eps_val = 1e-9;
    mu0 = params.physical.mu0;
    n_perm = length(params.permanent.r);
    n_sol  = length(params.solenoids.r);
    nEval  = size(px, 2);

    % --- Build SCALAR SX Function: one source, one eval point ---
    px_s  = SX.sym('px');
    py_s  = SX.sym('py');
    pz_s  = SX.sym('pz');
    src_x = SX.sym('src_x');
    src_y = SX.sym('src_y');
    src_z = SX.sym('src_z');
    src_r = SX.sym('src_r');
    src_l = SX.sym('src_l');
    src_I = SX.sym('src_I');

    dx_s  = px_s - src_x;
    dy_s  = py_s - src_y;
    dz_s  = pz_s - src_z;
    rho_s = sqrt(dx_s^2 + dy_s^2);

    switch modelId
        case MaglevModel.Fast
            [brho_s, bz_s] = computeFieldCircularWirePolar_casadi(...
                rho_s, dz_s, src_r, src_I, mu0);
        otherwise  % Accurate
            [brho_s, bz_s] = computeFieldCircularCurrentSheetPolar_casadi(...
                rho_s, dz_s, src_r, src_l, src_I, mu0);
    end

    bx_s = brho_s * dx_s / (rho_s + eps_val);
    by_s = brho_s * dy_s / (rho_s + eps_val);

    f_pt = Function('f_pt', ...
        {px_s, py_s, pz_s, src_x, src_y, src_z, src_r, src_l, src_I}, ...
        {bx_s, by_s, bz_s});

    % --- Permanent magnets: map over n_perm * nEval pairs ---
    n_total_perm = n_perm * nEval;
    f_perm = f_pt.map(n_total_perm);

    % Build input vectors: layout [src1_pt1, ..., src1_ptN, src2_pt1, ..., srcM_ptN]
    I_perm = params.permanent.J / mu0 * params.permanent.l;

    [bx_p, by_p, bz_p] = f_perm(...
        repmat(px, 1, n_perm), ...                              % eval points repeated per source
        repmat(py, 1, n_perm), ...
        repmat(pz, 1, n_perm), ...
        kron(params.permanent.x, ones(1, nEval)), ...           % source params repeated per eval point
        kron(params.permanent.y, ones(1, nEval)), ...
        kron(params.permanent.z, ones(1, nEval)), ...
        kron(params.permanent.r, ones(1, nEval)), ...
        kron(params.permanent.l, ones(1, nEval)), ...
        kron(I_perm, ones(1, nEval)));

    % Reshape (1, n_perm*nEval) -> (nEval, n_perm), sum over sources
    bx = sum2(reshape(bx_p, nEval, n_perm))';
    by = sum2(reshape(by_p, nEval, n_perm))';
    bz = sum2(reshape(bz_p, nEval, n_perm))';

    % --- Solenoids: map over n_sol * nEval pairs ---
    n_total_sol = n_sol * nEval;
    f_sol = f_pt.map(n_total_sol);

    % Effective current per solenoid: nw * u(i), symbolic
    % Repeat each element nEval times: [nw*u(1) × nEval, nw*u(2) × nEval, ...]
    I_sol_vec = (params.solenoids.nw * u)';                     % (1, n_sol) symbolic
    I_sol_rep = reshape(ones(nEval, 1) * I_sol_vec, 1, n_total_sol);  % (1, n_sol*nEval)

    [bx_sol, by_sol, bz_sol] = f_sol(...
        repmat(px, 1, n_sol), ...
        repmat(py, 1, n_sol), ...
        repmat(pz, 1, n_sol), ...
        kron(params.solenoids.x, ones(1, nEval)), ...
        kron(params.solenoids.y, ones(1, nEval)), ...
        kron(params.solenoids.z, ones(1, nEval)), ...
        kron(params.solenoids.r, ones(1, nEval)), ...
        kron(params.solenoids.l, ones(1, nEval)), ...
        I_sol_rep);

    bx = bx + sum2(reshape(bx_sol, nEval, n_sol))';
    by = by + sum2(reshape(by_sol, nEval, n_sol))';
    bz = bz + sum2(reshape(bz_sol, nEval, n_sol))';
end
