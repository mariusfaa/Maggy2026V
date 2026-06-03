function [ocp_solver, model_name, build_meta] = build_ocp(M, cfg)
% BUILD_OCP  Build an acados OCP solver from a validated cfg and model.
%
%   [ocp_solver, model_name] = build_ocp(M, cfg)
%   [ocp_solver, model_name, build_meta] = build_ocp(M, cfg)
%
% Inputs
%   M     struct from build_model()
%   cfg   validated cfg (call validate_cfg first). Every acados knob the
%         framework supports comes from cfg fields; no hidden defaults.
%
% Outputs
%   ocp_solver   AcadosOcpSolver
%   model_name   the (Windows-260-safe) acados model_name used for codegen
%   build_meta   struct with build_time_s and model_name for the result
%                record
%
% The cost is always NONLINEAR_LS with weights (Q, R) and terminal weight
% W_e = alpha_we * Q. Constraints follow the working baselines: input box
% [-1, 1] on all 4 channels; soft state box on the indices listed in
% cfg.idxbx, softened on cfg.idxsbx with Zl, Zu, zl, zu penalties.

    if nargin < 2; error('build_ocp:nargin','Usage: build_ocp(M, cfg)'); end

    % --- Session cache (cfg_hash -> {solver, meta}) -------------------
    %
    % Without this, repeated build_ocp calls with the same cfg in one
    % MATLAB session re-enter acados' codegen and try to overwrite the
    % loaded DLL, which fails on Windows ("Permission denied" at the
    % linker step). Two identical cfgs map to the same model_name, so
    % caching on cfg_hash is exact.
    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType','char','ValueType','any');
    end
    cfg_hash = hash_cfg(cfg);
    if isKey(cache, cfg_hash)
        entry = cache(cfg_hash);
        if isa(entry.solver,'AcadosOcpSolver') && isvalid(entry.solver)
            ocp_solver = entry.solver;
            model_name = entry.model_name;
            build_meta = entry.build_meta;
            return
        end
        remove(cache, cfg_hash);  % stale entry; rebuild
    end

    nu      = M.nu;
    nx_ctrl = M.nx_ctrl;

    % --- Cost matrices -------------------------------------------------
    Q   = as_matrix(cfg.Q, nx_ctrl, 'cfg.Q');
    R   = as_matrix(cfg.R, nu,      'cfg.R');
    W   = blkdiag(Q, R);
    W_e = cfg.alpha_we * Q;

    % --- Compose acados OCP -------------------------------------------
    ocp = AcadosOcp();
    model_name = make_model_name(M, cfg);
    ocp.model.name        = model_name;
    ocp.model.x           = M.x_ctrl_sym;
    ocp.model.u           = M.u_sym;
    ocp.model.xdot        = M.xdot_ctrl_sym;
    ocp.model.f_impl_expr = M.xdot_ctrl_sym - M.f_expl_ctrl;
    if strcmp(cfg.integrator_type,'ERK')
        ocp.model.f_expl_expr = M.f_expl_ctrl;
    end

    % --- Solver options (no hidden defaults) --------------------------
    ocp.solver_options.N_horizon              = cfg.N;
    ocp.solver_options.tf                     = cfg.Tf;
    ocp.solver_options.integrator_type        = cfg.integrator_type;
    ocp.solver_options.sim_method_num_stages  = cfg.sim_method_num_stages;
    ocp.solver_options.sim_method_num_steps   = cfg.sim_method_num_steps;
    ocp.solver_options.nlp_solver_type        = cfg.nlp_solver_type;
    ocp.solver_options.nlp_solver_max_iter    = cfg.nlp_solver_max_iter; % validate_cfg forces 1 for RTI
    ocp.solver_options.nlp_solver_tol_stat    = cfg.nlp_solver_tol_stat;
    ocp.solver_options.nlp_solver_tol_eq      = cfg.nlp_solver_tol_eq;
    ocp.solver_options.nlp_solver_tol_ineq    = cfg.nlp_solver_tol_ineq;
    ocp.solver_options.nlp_solver_tol_comp    = cfg.nlp_solver_tol_comp;
    ocp.solver_options.qp_solver              = cfg.qp_solver;
    ocp.solver_options.qp_solver_iter_max     = cfg.qp_solver_iter_max;
    ocp.solver_options.qp_solver_warm_start   = cfg.qp_solver_warm_start;
    ocp.solver_options.hessian_approx         = cfg.hessian_approx;
    ocp.solver_options.regularize_method      = cfg.regularize_method;

    % --- Cost: NONLINEAR_LS tracking xEq, uEq -------------------------
    ocp.cost.cost_type   = 'NONLINEAR_LS';
    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.cost_type_e = 'NONLINEAR_LS';
    ocp.cost.W           = W;
    ocp.cost.W_0         = W;
    ocp.cost.W_e         = W_e;
    ocp.model.cost_y_expr   = [M.x_ctrl_sym; M.u_sym];
    ocp.model.cost_y_expr_0 = [M.x_ctrl_sym; M.u_sym];
    ocp.model.cost_y_expr_e = M.x_ctrl_sym;
    ocp.cost.yref   = [M.xEq_ctrl; M.uEq];
    ocp.cost.yref_0 = [M.xEq_ctrl; M.uEq];
    ocp.cost.yref_e = M.xEq_ctrl;

    % --- Input box constraints ----------------------------------------
    ocp.constraints.idxbu = 0:(nu-1);
    ocp.constraints.lbu   = cfg.lbu;
    ocp.constraints.ubu   = cfg.ubu;

    % --- Soft state box constraints -----------------------------------
    ocp.constraints.idxbx  = cfg.idxbx(:).';
    ocp.constraints.lbx    = cfg.lbx(:);
    ocp.constraints.ubx    = cfg.ubx(:);
    ocp.constraints.idxsbx = cfg.idxsbx(:).';
    ocp.cost.Zl = cfg.Zl(:);
    ocp.cost.Zu = cfg.Zu(:);
    ocp.cost.zl = cfg.zl(:);
    ocp.cost.zu = cfg.zu(:);

    % --- Initial state placeholder (overwritten per step) -------------
    ocp.constraints.x0 = M.xEq_ctrl;

    % --- Compile -------------------------------------------------------
    t0 = tic;
    ocp_solver = AcadosOcpSolver(ocp);
    build_time_s = toc(t0);

    build_meta = struct( ...
        'model_name',       model_name, ...
        'build_time_s',     build_time_s, ...
        'asymmetric_plant', cfg.plant_override.is_asymmetric, ...
        'cfg_hash',         cfg_hash ...
    );

    cache(cfg_hash) = struct( ...
        'solver',     ocp_solver, ...
        'model_name', model_name, ...
        'build_meta', build_meta ...
    );
end

% ---------------------------------------------------------------------- %

function W = as_matrix(W_in, n, name)
% Accept either a length-n vector (diagonal) or an n-by-n matrix.
    if isempty(W_in)
        error('build_ocp:empty_weight', '%s is empty', name);
    end
    if isvector(W_in)
        if numel(W_in) ~= n
            error('build_ocp:wrong_dim', ...
                  '%s has %d entries, expected %d', name, numel(W_in), n);
        end
        W = diag(W_in(:));
    elseif ismatrix(W_in)
        if any(size(W_in) ~= [n n])
            error('build_ocp:wrong_dim', ...
                  '%s has size %dx%d, expected %dx%d', ...
                  name, size(W_in,1), size(W_in,2), n, n);
        end
        W = W_in;
    else
        error('build_ocp:bad_type', '%s must be a vector or matrix', name);
    end
end

function name = make_model_name(M, cfg)
% Short, Windows-260-path-safe acados model_name. The full cfg_hash is
% truncated to 12 hex chars; two cfgs differing in any compiled-in option
% collide only with probability ~2e-15. Distinct models go in distinct
% c_generated_code/<name>/ trees, so this is the only guard against
% silent cache collisions.
    mc = lower(M.order(1));         % 'f' or 'r'
    stage_clean = regexprep(cfg.stage, '[^A-Za-z0-9]', '');
    if numel(stage_clean) > 6; stage_clean = stage_clean(1:6); end
    h = hash_cfg(cfg);
    h12 = h(1:12);
    name = sprintf('m_%s_%s_%s', stage_clean, mc, h12);
    name = regexprep(name, '[^A-Za-z0-9_]', '_');
end
