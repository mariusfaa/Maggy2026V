function M = build_model(model_kind, controller_n, plant_n)
% BUILD_MODEL  Build controller and plant CasADi expressions plus
% equilibrium and projection helpers.
%
%   M = build_model('reduced10', 10, 30)
%   M = build_model('full12',     20, 30)
%
% Inputs
%   model_kind     'reduced10' (10-state OCP) or 'full12' (12-state OCP)
%   controller_n   levitating-magnet discretization for the OCP dynamics
%   plant_n        same for the plant simulator (default 30)
%
% Output struct M
%   .nu                 4
%   .nx_ctrl            10 or 12
%   .nx_plant           12 (always)
%   .x_ctrl_sym         SX.sym, dim nx_ctrl   (input slot for ocp.model.x)
%   .xdot_ctrl_sym      SX.sym, dim nx_ctrl   (slot for ocp.model.xdot)
%   .x_plant_sym        SX.sym, dim 12        (slot for sim.model.x)
%   .xdot_plant_sym     SX.sym, dim 12        (slot for sim.model.xdot)
%   .u_sym              SX.sym, dim 4
%   .f_expl_ctrl        controller-side CasADi expression in (x_ctrl_sym, u_sym)
%   .f_expl_plant       plant-side CasADi expression in (x_plant_sym, u_sym)
%   .xEq_plant          12-dim equilibrium state
%   .xEq_ctrl           equilibrium projected to controller-side dim
%   .uEq                4x1 zeros
%   .params_ctrl        params struct used for controller (with magnet.n=controller_n)
%   .params_plant       params struct used for plant
%   .project_to_ctrl(x12)   -> x_ctrl   (handle, numeric)
%   .lift_to_plant(x_ctrl)  -> x12      (handle, numeric; yaw, wz set to 0)
%   .ctrl_idx           indices into the 12-state vector that survive the
%                       projection (10-state: [1 2 3 4 5 7 8 9 10 11])

    import casadi.*

    if nargin < 3 || isempty(plant_n); plant_n = 30; end

    nu = 4;

    % --- Build dynamics for the two discretizations ---
    [f_func_ctrl,  params_ctrl]  = build_dynamics(controller_n);
    if controller_n == plant_n
        f_func_plant = f_func_ctrl;
        params_plant = params_ctrl;
    else
        [f_func_plant, params_plant] = build_dynamics(plant_n);
    end

    % --- Equilibrium from plant truth (12-state, gamma=wz=0) ---
    z_var = SX.sym('z_eq');
    x_eq_sym = [0; 0; z_var; zeros(9,1)];
    accel = f_func_plant(x_eq_sym, zeros(nu,1));
    nlp = struct('x', z_var, 'f', accel(9)^2);
    sol_eq = nlpsol('eq', 'ipopt', nlp, struct('ipopt', struct('print_level', 0)));
    sol = sol_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
    zEq = full(sol.x);

    xEq_plant = [0; 0; zEq; zeros(9,1)];
    uEq = zeros(nu, 1);

    % --- Symbols for plant & controller sides ---
    x_plant_sym    = SX.sym('x_plant', 12);
    xdot_plant_sym = SX.sym('xdot_plant', 12);
    u_sym          = SX.sym('u', nu);
    f_expl_plant   = f_func_plant(x_plant_sym, u_sym);

    switch lower(model_kind)
        case 'reduced10'
            nx_ctrl = 10;
            ctrl_idx = [1 2 3 4 5 7 8 9 10 11];
            x_ctrl_sym    = SX.sym('x_ctrl', nx_ctrl);
            xdot_ctrl_sym = SX.sym('xdot_ctrl', nx_ctrl);
            % lift to 12-state with yaw=0, wz=0 for the controller's internal sim
            xfull_from_ctrl = [x_ctrl_sym(1:5); 0; x_ctrl_sym(6:8); x_ctrl_sym(9:10); 0];
            dx_full = f_func_ctrl(xfull_from_ctrl, u_sym);
            f_expl_ctrl = dx_full(ctrl_idx);
            xEq_ctrl = xEq_plant(ctrl_idx);

            project_to_ctrl = @(x12) x12(ctrl_idx);
            lift_to_plant   = @(x10) [x10(1:5); 0; x10(6:8); x10(9:10); 0];

        case 'full12'
            nx_ctrl = 12;
            ctrl_idx = 1:12;
            x_ctrl_sym    = SX.sym('x_ctrl', nx_ctrl);
            xdot_ctrl_sym = SX.sym('xdot_ctrl', nx_ctrl);
            f_expl_ctrl = f_func_ctrl(x_ctrl_sym, u_sym);
            xEq_ctrl = xEq_plant;

            project_to_ctrl = @(x12) x12;
            lift_to_plant   = @(x12) x12;

        otherwise
            error('build_model:bad_kind', ...
                'Unknown model_kind ''%s'' (expected ''reduced10'' or ''full12'').', model_kind);
    end

    % --- Pack ---
    M.nu              = nu;
    M.nx_ctrl         = nx_ctrl;
    M.nx_plant        = 12;
    M.x_ctrl_sym      = x_ctrl_sym;
    M.xdot_ctrl_sym   = xdot_ctrl_sym;
    M.x_plant_sym     = x_plant_sym;
    M.xdot_plant_sym  = xdot_plant_sym;
    M.u_sym           = u_sym;
    M.f_expl_ctrl     = f_expl_ctrl;
    M.f_expl_plant    = f_expl_plant;
    M.xEq_plant       = xEq_plant;
    M.xEq_ctrl        = xEq_ctrl;
    M.uEq             = uEq;
    M.params_ctrl     = params_ctrl;
    M.params_plant    = params_plant;
    M.project_to_ctrl = project_to_ctrl;
    M.lift_to_plant   = lift_to_plant;
    M.ctrl_idx        = ctrl_idx;
    M.model_kind      = lower(model_kind);
    M.controller_n    = controller_n;
    M.plant_n         = plant_n;
end
