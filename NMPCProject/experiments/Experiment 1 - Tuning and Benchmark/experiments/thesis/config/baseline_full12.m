function cfg = baseline_full12()
% BASELINE_FULL12  Canonical cfg that reproduces workingSimulator.m.
%
%   cfg = baseline_full12()
%
% Solver:    SQP_RTI, PARTIAL_CONDENSING_HPIPM, IRK 4 stages / 10 steps,
%            GAUSS_NEWTON Hessian, CONVEXIFY regularization.
% Horizon:   N = 20 nodes, Tf = 0.20 s (dt = 10 ms).
% Cost:      diag([1e2 1e2 1e3  1e3 1e3 1e1  1e1 1e1 1e1  1e1 1e1 1e0])
%            R = eye(4), terminal multiplier alpha_we = 10.
% Slacks:    Zl=Zu=1e3, zl=zu=1e2 on the 5 soft state-box constraints.
% IC:        perturbation_scale 0.05 along the baseline direction (12-vec).
%
% Plant integrator: matches the OCP (matched-plant policy). The original
% workingSimulator.m already had matched OCP/plant integrators (both IRK
% 4/10), so this cfg reproduces it bit-exactly without any plant_override.

    cfg = struct();

    cfg.stage  = 'A';
    cfg.exp_id = 'baseline_full12';
    cfg.order  = 'full12';

    cfg.N      = 20;
    cfg.Tf     = 0.20;

    cfg.integrator_type        = 'IRK';
    cfg.sim_method_num_stages  = 4;
    cfg.sim_method_num_steps   = 10;

    cfg.nlp_solver_type   = 'SQP_RTI';
    cfg.qp_solver         = 'PARTIAL_CONDENSING_HPIPM';
    cfg.hessian_approx    = 'GAUSS_NEWTON';
    cfg.regularize_method = 'CONVEXIFY';

    cfg.qp_solver_iter_max   = 200;
    cfg.qp_solver_warm_start = 1;
    cfg.nlp_solver_tol_stat  = 1e-4;
    cfg.nlp_solver_tol_eq    = 1e-4;
    cfg.nlp_solver_tol_ineq  = 1e-4;
    cfg.nlp_solver_tol_comp  = 1e-4;

    cfg.Q = [1e2 1e2 1e3, ...   % x, y, z
             1e3 1e3 1e1, ...   % roll, pitch, yaw
             1e1 1e1 1e1, ...   % vx, vy, vz
             1e1 1e1 1e0];      % wx, wy, wz
    cfg.R         = [1 1 1 1];
    cfg.alpha_we  = 10;

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

    cfg.sim_steps          = 200;
    cfg.ic_descriptor      = struct( ...
        'scale',     0.05, ...
        'direction', [0.005; -0.008; 0.010;  0.15; -0.10; 0.3; ...
                      0.01;  -0.01;   0.0;   0.2;  -0.3;  0.0]);
    cfg.warmstart_strategy = 'linear_to_eq';

    cfg.disturbance_schedule = [];
end
