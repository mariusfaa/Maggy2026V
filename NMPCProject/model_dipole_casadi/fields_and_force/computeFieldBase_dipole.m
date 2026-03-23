function [bx, by, bz] = computeFieldBase_dipole(px, py, pz, u, params)
% COMPUTEFIELDBASE_DIPOLE  Total field from 4 permanent magnets + 4 solenoids.
%
% Each source is modeled as a magnetic dipole oriented along z (aligned with
% the base plane normal). Uses CasADi SX Functions with map() for efficiency.
%
% Permanent magnets: moment = (J * Volume / mu0) * zhat  (fixed, no u dependence)
% Solenoids:         moment = (nw * u_i) * A * zhat      (current-dependent)
%   where A = pi * r^2 is the solenoid cross-section area.
%
% Inputs:
%   px, py, pz - Field point coordinates (1 x nEval, MX)
%   u          - Solenoid currents (4 x 1, MX)
%   params     - Parameter struct

    import casadi.*

    mu0    = params.physical.mu0;
    n_perm = length(params.permanent.r);
    n_sol  = length(params.solenoids.r);
    nEval  = size(px, 2);

    % --- Permanent magnets (dipole moment from magnetization) ---
    % m = M * V = (J/mu0) * (pi * r^2 * l) * zhat
    % J is the remanence (T), so M = J/mu0
    bx_acc = zeros(1, nEval);
    by_acc = zeros(1, nEval);
    bz_acc = zeros(1, nEval);

    for i = 1:n_perm
        r_i = params.permanent.r(i);
        l_i = params.permanent.l(i);
        J_i = params.permanent.J;
        V_i = pi * r_i^2 * l_i;
        m_perm = (J_i / mu0) * V_i;  % scalar dipole moment along z

        % Build SX function: f(px, py, pz) -> (bx, by, bz)
        px_s = SX.sym('px');
        py_s = SX.sym('py');
        pz_s = SX.sym('pz');

        [bx_s, by_s, bz_s] = computeFieldDipole_casadi(px_s, py_s, pz_s, ...
            params.permanent.x(i), params.permanent.y(i), params.permanent.z(i), ...
            0, 0, m_perm, mu0);

        f_p = Function(sprintf('f_dip_perm%d', i), ...
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

    % --- Solenoids (dipole moment = nw * I * A * zhat) ---
    % A = pi * r^2, I = u(i)
    for i = 1:n_sol
        r_i = params.solenoids.r(i);
        A_i = pi * r_i^2;
        % m_sol = nw * u(i) * A_i  (symbolic in u)

        px_s = SX.sym('px');
        py_s = SX.sym('py');
        pz_s = SX.sym('pz');
        I_s  = SX.sym('I');

        mz_sol = I_s * A_i;  % nw factored into I_s at call site

        [bx_s, by_s, bz_s] = computeFieldDipole_casadi(px_s, py_s, pz_s, ...
            params.solenoids.x(i), params.solenoids.y(i), params.solenoids.z(i), ...
            0, 0, mz_sol, mu0);

        f_s = Function(sprintf('f_dip_sol%d', i), ...
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
