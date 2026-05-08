function [ocp_solver, model_name] = build_ocp(cfg, M)
% BUILD_OCP  Build an acados OCP solver from a single cfg struct.
%
%   [ocp_solver, model_name] = build_ocp(cfg, M)
%
% Required cfg fields:
%   .stage, .exp_id            string identifiers (used in model_name)
%   .N, .Tf                    horizon nodes and length
%   .integ_type                'ERK' | 'IRK' | 'GNSF'
%   .n_stages, .n_steps        sim_method_num_stages / num_steps
%   .nlp_solver                'SQP' | 'SQP_RTI'
%   .qp_solver                 'FULL_CONDENSING_HPIPM' | 'PARTIAL_CONDENSING_HPIPM' | 'FULL_CONDENSING_QPOASES'
%   .hessian_approx            'GAUSS_NEWTON' | 'EXACT'
%   .Q                         nx_ctrl x nx_ctrl  (or vector of diag entries)
%   .R                         nu x nu (or vector)
%   .alpha_we                  scalar terminal-cost multiplier (W_e = alpha_we * Q)
%
% Optional cfg fields (with defaults):
%   .qp_solver_iter_max        200
%   .qp_solver_warm_start      1
%   .nlp_solver_tol_*          1e-4
%   .regularize_method         'CONVEXIFY'
%   .lbx_idx                   constraint indices (10-state default: 0:4)
%   .lbx                       lower box on constrained states
%   .ubx                       upper box
%   .Zl, .Zu, .zl, .zu         soft-constraint penalties
%
% Returns the AcadosOcpSolver and the model_name actually used (so the
% caller can clean up c_generated_code/<model_name>/ later if needed).
%
% Critical fix vs the legacy runExperimentB build_ocp helper: the
% legacy code unconditionally set nlp_solver_max_iter=1, which makes a
% configuration with .nlp_solver='SQP' silently behave as SQP_RTI. This
% function gates the iter cap on .nlp_solver.

    import casadi.*

    nu      = M.nu;
    nx_ctrl = M.nx_ctrl;

    % --- Defaults ---
    cfg = setDefault(cfg, 'qp_solver_iter_max',   200);
    cfg = setDefault(cfg, 'qp_solver_warm_start', 1);
    cfg = setDefault(cfg, 'nlp_solver_tol_stat',  1e-4);
    cfg = setDefault(cfg, 'nlp_solver_tol_eq',    1e-4);
    cfg = setDefault(cfg, 'nlp_solver_tol_ineq',  1e-4);
    cfg = setDefault(cfg, 'nlp_solver_tol_comp',  1e-4);
    cfg = setDefault(cfg, 'regularize_method',    'CONVEXIFY');

    % Default soft-state-constraint indices: position (xy), z, roll, pitch.
    cfg = setDefault(cfg, 'lbx_idx_zero_based', [0 1 2 3 4]);
    cfg = setDefault(cfg, 'lbx',                [-0.025; -0.025; 0.015; -0.35; -0.35]);
    cfg = setDefault(cfg, 'ubx',                [ 0.025;  0.025; 0.055;  0.35;  0.35]);
    n_sbx = length(cfg.lbx_idx_zero_based);
    cfg = setDefault(cfg, 'Zl', 1e3 * ones(n_sbx,1));
    cfg = setDefault(cfg, 'Zu', 1e3 * ones(n_sbx,1));
    cfg = setDefault(cfg, 'zl', 1e2 * ones(n_sbx,1));
    cfg = setDefault(cfg, 'zu', 1e2 * ones(n_sbx,1));

    % --- Cost matrices: accept diagonal vectors or full matrices ---
    Q = asMatrix(cfg.Q, nx_ctrl);
    R = asMatrix(cfg.R, nu);
    W_e = cfg.alpha_we * Q;

    % --- Unique model name (avoids acados c_generated_code cache collisions) ---
    model_name = makeModelName(cfg, M);

    % --- Build OCP ---
    ocp = AcadosOcp();
    ocp.model.name        = model_name;
    ocp.model.x           = M.x_ctrl_sym;
    ocp.model.u           = M.u_sym;
    ocp.model.xdot        = M.xdot_ctrl_sym;
    ocp.model.f_impl_expr = M.xdot_ctrl_sym - M.f_expl_ctrl;
    ocp.model.f_expl_expr = M.f_expl_ctrl;

    ocp.solver_options.N_horizon             = cfg.N;
    ocp.solver_options.tf                    = cfg.Tf;
    ocp.solver_options.integrator_type       = cfg.integ_type;
    ocp.solver_options.sim_method_num_stages = cfg.n_stages;
    ocp.solver_options.sim_method_num_steps  = cfg.n_steps;
    ocp.solver_options.nlp_solver_type       = cfg.nlp_solver;

    % --- nlp_solver_max_iter: gate on solver type (the bug fix) ---
    if strcmpi(cfg.nlp_solver, 'SQP')
        cfg = setDefault(cfg, 'nlp_solver_max_iter', 50);
    else
        cfg = setDefault(cfg, 'nlp_solver_max_iter', 1);
    end
    ocp.solver_options.nlp_solver_max_iter = cfg.nlp_solver_max_iter;

    ocp.solver_options.nlp_solver_tol_stat = cfg.nlp_solver_tol_stat;
    ocp.solver_options.nlp_solver_tol_eq   = cfg.nlp_solver_tol_eq;
    ocp.solver_options.nlp_solver_tol_ineq = cfg.nlp_solver_tol_ineq;
    ocp.solver_options.nlp_solver_tol_comp = cfg.nlp_solver_tol_comp;
    ocp.solver_options.qp_solver           = cfg.qp_solver;
    ocp.solver_options.qp_solver_iter_max  = cfg.qp_solver_iter_max;
    ocp.solver_options.qp_solver_warm_start = cfg.qp_solver_warm_start;
    ocp.solver_options.hessian_approx      = cfg.hessian_approx;
    ocp.solver_options.regularize_method   = cfg.regularize_method;

    % --- Cost (NONLINEAR_LS, weights on x and u) ---
    ocp.cost.cost_type   = 'NONLINEAR_LS';
    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.cost_type_e = 'NONLINEAR_LS';
    ocp.cost.W           = blkdiag(Q, R);
    ocp.cost.W_0         = blkdiag(Q, R);
    ocp.cost.W_e         = W_e;
    ocp.model.cost_y_expr   = [M.x_ctrl_sym; M.u_sym];
    ocp.model.cost_y_expr_0 = [M.x_ctrl_sym; M.u_sym];
    ocp.model.cost_y_expr_e = M.x_ctrl_sym;
    ocp.cost.yref   = [M.xEq_ctrl; M.uEq];
    ocp.cost.yref_0 = [M.xEq_ctrl; M.uEq];
    ocp.cost.yref_e = M.xEq_ctrl;

    % --- Box constraints on inputs ---
    ocp.constraints.idxbu = 0:nu-1;
    ocp.constraints.lbu   = -ones(nu, 1);
    ocp.constraints.ubu   =  ones(nu, 1);

    % --- Soft state-box constraints ---
    ocp.constraints.idxbx  = cfg.lbx_idx_zero_based;
    ocp.constraints.lbx    = cfg.lbx;
    ocp.constraints.ubx    = cfg.ubx;
    ocp.constraints.idxsbx = 0:(n_sbx-1);
    ocp.cost.Zl = cfg.Zl;
    ocp.cost.Zu = cfg.Zu;
    ocp.cost.zl = cfg.zl;
    ocp.cost.zu = cfg.zu;

    ocp.constraints.x0 = M.xEq_ctrl;

    ocp_solver = AcadosOcpSolver(ocp);
end

% =========================================================================
% Helpers
% =========================================================================

function s = setDefault(s, field, val)
    if ~isfield(s, field) || isempty(s.(field))
        s.(field) = val;
    end
end

function W = asMatrix(W_in, n)
    if isempty(W_in)
        error('build_ocp:emptyWeight', 'Weight matrix is empty.');
    end
    if isvector(W_in)
        if numel(W_in) ~= n
            error('build_ocp:wrongDim', ...
                'Diagonal vector length %d != expected %d.', numel(W_in), n);
        end
        W = diag(W_in(:));
    else
        W = W_in;
    end
end

function name = makeModelName(cfg, M)
% Deterministic short-but-unique name. Acados caches by model_name in
% c_generated_code/<name>/ so different cfgs MUST have different names.
    integ = lower(cfg.integ_type);
    nlp   = lower(strrep(cfg.nlp_solver, 'SQP_RTI', 'rti'));
    nlp   = strrep(nlp, 'sqp', 'sqp');
    qp    = lower(cfg.qp_solver);
    qp    = strrep(qp, 'full_condensing_hpipm',    'fch');
    qp    = strrep(qp, 'partial_condensing_hpipm', 'pch');
    qp    = strrep(qp, 'full_condensing_qpoases',  'fcq');
    hess  = lower(cfg.hessian_approx);
    hess  = strrep(hess, 'gauss_newton', 'gn');
    hess  = strrep(hess, 'exact',        'ex');

    Tf_ms = round(cfg.Tf * 1000);
    name  = sprintf('mlev_%s_%s_%s_n%d_N%d_T%d_%s_%d_%d_%s_%s_%s', ...
        cfg.stage, cfg.exp_id, M.model_kind, M.controller_n, ...
        cfg.N, Tf_ms, integ, cfg.n_stages, cfg.n_steps, nlp, qp, hess);
    name = sanitize(name);
end

function s = sanitize(s)
    s = regexprep(s, '[^A-Za-z0-9_]', '_');
    if length(s) > 60
        s = sprintf('%s_h%s', s(1:50), dec2hex(mod(string2hash(s), 1e6)));
    end
end

function h = string2hash(s)
    h = uint32(0);
    for i = 1:length(s)
        h = mod(h*uint32(31) + uint32(s(i)), uint32(2^32 - 1));
    end
    h = double(h);
end
