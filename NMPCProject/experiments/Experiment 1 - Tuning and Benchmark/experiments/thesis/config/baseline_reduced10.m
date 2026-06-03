function cfg = baseline_reduced10()
% BASELINE_REDUCED10  Canonical cfg reproducing workingSimulatorReducedOrder.m.
%
%   cfg = baseline_reduced10()
%
% Reduced 10-state controller (yaw decoupled out). N = 10, Tf = 0.10 s.
%
% Solver:    SQP_RTI, FULL_CONDENSING_HPIPM, IRK 2 stages / 5 steps,
%            GAUSS_NEWTON Hessian, CONVEXIFY regularization.
% Cost:      diag([1e2 1e2 1e3  1e3 1e3  1e1 1e1 1e1  1e1 1e1])
%            R = eye(4), terminal multiplier alpha_we = 50.
%
% PLANT_OVERRIDE -- THE ONE DOCUMENTED EXCEPTION TO MATCHED-PLANT POLICY.
% The original workingSimulatorReducedOrder.m runs the OCP at IRK 2/5 while
% the plant runs at IRK 4/10 (more accurate truth). This cfg reproduces
% that asymmetry by explicitly setting cfg.plant_override.* with a reason
% string. The framework flags this run as asymmetric_plant=true.
%
% Disturbance: z-axis push at t = 5.0 s with magnitude
%   [0; 0; 0; 0; 0; -0.005; 0.0025; -0.2; 0; 0]   (10-vector on x_ctrl)
% (which, lifted to 12-state, is [0;0;0; 0;0;0;  -0.005; 0.0025; -0.2;  0;0;0]).

    cfg = struct();

    cfg.stage  = 'A';
    cfg.exp_id = 'baseline_reduced10';
    cfg.order  = 'reduced10';

    cfg.N      = 10;
    cfg.Tf     = 0.10;

    cfg.integrator_type        = 'IRK';
    cfg.sim_method_num_stages  = 2;
    cfg.sim_method_num_steps   = 5;

    cfg.nlp_solver_type   = 'SQP_RTI';
    cfg.qp_solver         = 'FULL_CONDENSING_HPIPM';
    cfg.hessian_approx    = 'GAUSS_NEWTON';
    cfg.regularize_method = 'CONVEXIFY';

    cfg.qp_solver_iter_max   = 200;
    cfg.qp_solver_warm_start = 1;
    cfg.nlp_solver_tol_stat  = 1e-4;
    cfg.nlp_solver_tol_eq    = 1e-4;
    cfg.nlp_solver_tol_ineq  = 1e-4;
    cfg.nlp_solver_tol_comp  = 1e-4;

    cfg.Q = [1e2 1e2 1e3, ...   % x, y, z
             1e3 1e3, ...        % roll, pitch (yaw dropped)
             1e1 1e1 1e1, ...   % vx, vy, vz
             1e1 1e1];           % wx, wy (wz dropped)
    cfg.R        = [1 1 1 1];
    cfg.alpha_we = 50;

    cfg.lbu    = -ones(4,1);
    cfg.ubu    =  ones(4,1);
    cfg.idxbx  = [0 1 2 3 4];
    cfg.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
    cfg.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
    cfg.idxsbx = [0 1 2 3 4];
    cfg.Zl     = 1e3 * ones(5,1);
    cfg.Zu     = 1e3 * ones(5,1);
    cfg.zl     = 1e2 * ones(5,1);
    cfg.zu     = 1e2 * ones(5,1);

    cfg.sim_steps     = 1000;
    %
    % Direction is the 12-state perturbation applied to the plant. To
    % reproduce workingSimulatorReducedOrder.m's 10-element direction
    %   [x, y, z, roll, pitch, vx, vy, vz, wx, wy]
    % = [0.005, -0.008, 0.010, 0.15, -0.10, 0.01, -0.01, 0.0, 0.2, -0.3],
    % the yaw and yaw-rate slots (12-state indices 6 and 12) are set to
    % zero -- those DOFs are uncontrollable and intentionally NOT perturbed.
    cfg.ic_descriptor = struct( ...
        'scale',     0.05, ...
        'direction', [0.005; -0.008; 0.010; 0.15; -0.10; 0.0; ...
                      0.01;  -0.01;   0.0;  0.2;  -0.3;  0.0]);
    cfg.warmstart_strategy = 'linear_to_eq';

    % --- Plant override (documented asymmetry) -----------------------
    cfg.plant_override.integrator_type       = 'IRK';
    cfg.plant_override.sim_method_num_stages = 4;
    cfg.plant_override.sim_method_num_steps  = 10;
    cfg.plant_override.magnet_n              = 10;
    cfg.plant_override.reason = ['Reproduces workingSimulatorReducedOrder.m: ' ...
        'OCP IRK 2/5 against a higher-fidelity IRK 4/10 plant. ' ...
        'Used only for Stage A baseline validation.'];

    % --- Disturbance schedule ----------------------------------------
    cfg.disturbance_schedule = struct( ...
        't_apply', 5.0, ...
        'kind',    'state_additive', ...
        'vector',  [0; 0; 0; 0; 0; 0; ...
                    -0.005; 0.0025; -0.2; ...
                    0; 0; 0]);   % 12-element vector applied to plant state
end
