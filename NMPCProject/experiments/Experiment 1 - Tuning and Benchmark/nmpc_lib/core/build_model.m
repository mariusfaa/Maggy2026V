function M = build_model(opts)
% BUILD_MODEL  Build controller and plant symbolic expressions, equilibria,
% and projection helpers.
%
%   M = build_model(opts)
%
% opts (all fields optional):
%   .order            'full12' (default) | 'reduced10'
%   .magnet_n_plant   integer >= 3 (default 10 -- baseline)
%   .magnet_n_ctrl    integer >= 3 (default = magnet_n_plant)
%   .params           params struct (default: parameters_maggy_V4)
%
% Output (struct M)
%   .nu               4
%   .nx_ctrl          10 (reduced10) or 12 (full12)
%   .nx_plant         12 (always)
%   .order            'full12' | 'reduced10'
%   .magnet_n_ctrl, .magnet_n_plant
%
%   .x_ctrl_sym       SX.sym('x_ctrl', nx_ctrl)
%   .xdot_ctrl_sym    SX.sym('xdot_ctrl', nx_ctrl)
%   .x_plant_sym      SX.sym('x_plant', 12)
%   .xdot_plant_sym   SX.sym('xdot_plant', 12)
%   .u_sym            SX.sym('u', 4)
%   .f_expl_ctrl      controller-side dynamics in (x_ctrl_sym, u_sym)
%   .f_expl_plant     plant-side dynamics in (x_plant_sym, u_sym)
%
%   .xEq_plant        12-element equilibrium state
%   .xEq_ctrl         equilibrium projected to controller-side dimension
%   .uEq              4-element zero input
%   .equilibrium      full eq struct from compute_equilibrium (residual, hash...)
%
%   .params_ctrl      params struct used for controller dynamics
%   .params_plant     params struct used for plant dynamics
%
%   .project_to_ctrl(x12)  ->  x_ctrl       (numeric handle)
%   .lift_to_plant(x_ctrl) ->  x12          (yaw=0, wz=0 for reduced10)
%   .ctrl_idx          indices in the 12-state vector retained on the
%                       controller side (reduced10: [1 2 3 4 5 7 8 9 10 11])
%
% The matched-plant policy (plant integrator follows OCP integrator) is
% applied in build_plant; here we only build the SYMBOLIC dynamics.

    import casadi.*

    if nargin < 1; opts = struct(); end
    opts = with_default(opts, 'order',          'full12');
    opts = with_default(opts, 'magnet_n_plant', 10);
    opts = with_default(opts, 'magnet_n_ctrl',  opts.magnet_n_plant);
    opts = with_default(opts, 'params',         []);

    order = lower(opts.order);
    if ~ismember(order, {'full12','reduced10'})
        error('build_model:bad_order', ...
              'opts.order must be "full12" or "reduced10" (got "%s")', opts.order);
    end

    % --- Build dynamics for ctrl and plant (cached internally) -----------
    dyn_plant = build_dynamics(opts.magnet_n_plant, opts.params);
    if opts.magnet_n_ctrl == opts.magnet_n_plant
        dyn_ctrl = dyn_plant;
    else
        dyn_ctrl = build_dynamics(opts.magnet_n_ctrl, opts.params);
    end

    % --- Canonical equilibrium from PLANT dynamics (the ground truth) ---
    eq = compute_equilibrium(dyn_plant);

    % --- Symbolic vars for plant side ------------------------------------
    nu             = 4;
    x_plant_sym    = SX.sym('x_plant',    12);
    xdot_plant_sym = SX.sym('xdot_plant', 12);
    u_sym          = SX.sym('u',          nu);
    f_expl_plant   = dyn_plant.f_func(x_plant_sym, u_sym);

    % --- Controller side: choose order ----------------------------------
    switch order
        case 'reduced10'
            nx_ctrl  = 10;
            ctrl_idx = [1 2 3 4 5 7 8 9 10 11];

            x_ctrl_sym    = SX.sym('x_ctrl',    nx_ctrl);
            xdot_ctrl_sym = SX.sym('xdot_ctrl', nx_ctrl);

            % Lift 10-state to 12-state with yaw=0, wz=0 for ctrl-side dyn.
            xfull_from_ctrl = [x_ctrl_sym(1:5); 0; ...
                               x_ctrl_sym(6:8); ...
                               x_ctrl_sym(9:10); 0];
            dx_full = dyn_ctrl.f_func(xfull_from_ctrl, u_sym);
            f_expl_ctrl = dx_full(ctrl_idx);

            xEq_ctrl  = eq.xEq(ctrl_idx);
            project_h = @(x12)  x12(ctrl_idx);
            lift_h    = @(x10)  [x10(1:5); 0; x10(6:8); x10(9:10); 0];

        case 'full12'
            nx_ctrl  = 12;
            ctrl_idx = 1:12;

            x_ctrl_sym    = SX.sym('x_ctrl',    nx_ctrl);
            xdot_ctrl_sym = SX.sym('xdot_ctrl', nx_ctrl);
            f_expl_ctrl   = dyn_ctrl.f_func(x_ctrl_sym, u_sym);

            xEq_ctrl  = eq.xEq;
            project_h = @(x12) x12;
            lift_h    = @(x12) x12;
    end

    % --- Pack ----------------------------------------------------------
    M = struct();
    M.nu               = nu;
    M.nx_ctrl          = nx_ctrl;
    M.nx_plant         = 12;
    M.order            = order;
    M.magnet_n_ctrl    = opts.magnet_n_ctrl;
    M.magnet_n_plant   = opts.magnet_n_plant;

    M.x_ctrl_sym       = x_ctrl_sym;
    M.xdot_ctrl_sym    = xdot_ctrl_sym;
    M.x_plant_sym      = x_plant_sym;
    M.xdot_plant_sym   = xdot_plant_sym;
    M.u_sym            = u_sym;
    M.f_expl_ctrl      = f_expl_ctrl;
    M.f_expl_plant     = f_expl_plant;

    M.xEq_plant        = eq.xEq;
    M.xEq_ctrl         = xEq_ctrl;
    M.uEq              = eq.uEq;
    M.equilibrium      = eq;

    M.params_ctrl      = dyn_ctrl.params;
    M.params_plant     = dyn_plant.params;

    M.project_to_ctrl  = project_h;
    M.lift_to_plant    = lift_h;
    M.ctrl_idx         = ctrl_idx;
end

function s = with_default(s, fn, v)
    if ~isfield(s, fn) || isempty(s.(fn)); s.(fn) = v; end
end
