%% Quick self-test: Accurate ctrl + Accurate plant, umax=1

simSetup;
import casadi.*

umax = 1;
nx = 10;

params_acc = load_params(MaglevModel.Accurate);
[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);
xEq = [0; 0; zEq_acc(1); zeros(7,1)];
uEq = zeros(nu, 1);

x0_full = [0; 0; zEq_acc(1); zeros(9,1)] + [0; 0.001; 0.001; 0; 0; 0; zeros(6,1)];
x0 = x0_full([1:5,7:11]);

model = get_maggy_model(MaglevModel.Accurate, use_luts=true);

dt_mpc = 0.001; N_horizon = 40; Tf = dt_mpc * N_horizon;

ocp = AcadosOcp();
ocp.model = model;
ocp.solver_options.N_horizon = N_horizon;
ocp.solver_options.tf = Tf;
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps = 1;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.nlp_solver_max_iter = 200;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
ocp.solver_options.ext_fun_compile_flags = '-O2';

Q = diag([1e4,1e4,1e6,1e3,1e3,1e2,1e2,1e3,1e2,1e2]);
R = eye(nu) * 1e0;
ocp.cost.cost_type = 'LINEAR_LS'; ocp.cost.cost_type_0 = 'LINEAR_LS'; ocp.cost.cost_type_e = 'LINEAR_LS';
ocp.cost.Vx = [eye(nx); zeros(nu,nx)]; ocp.cost.Vu = [zeros(nx,nu); eye(nu)];
ocp.cost.Vx_0 = ocp.cost.Vx; ocp.cost.Vu_0 = ocp.cost.Vu; ocp.cost.Vx_e = eye(nx);
ocp.cost.W = blkdiag(Q,R); ocp.cost.W_0 = ocp.cost.W; ocp.cost.W_e = 10*Q;
ocp.cost.yref = [xEq;uEq]; ocp.cost.yref_0 = [xEq;uEq]; ocp.cost.yref_e = xEq;
ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu = -umax*ones(nu,1); ocp.constraints.ubu = umax*ones(nu,1);
ocp.constraints.x0 = x0;

ocp_solver = AcadosOcpSolver(ocp);
for k=0:N_horizon, ocp_solver.set('x', xEq, k); end
for k=0:N_horizon-1, ocp_solver.set('u', uEq, k); end

sim = AcadosSim(); sim.model = model;
sim.solver_options.Tsim = dt; sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages = 4; sim.solver_options.num_steps = 1;
sim_solver = AcadosSimSolver(sim);

x = x0; u = uEq;
fprintf('\n--- Accurate self-test (umax=1) ---\n');
for k = 1:200
    ocp_solver.set('constr_x0', x);
    ocp_solver.solve();
    u = ocp_solver.get('u', 0);
    for i=0:N_horizon-2
        ocp_solver.set('x', ocp_solver.get('x',i+1), i);
        ocp_solver.set('u', ocp_solver.get('u',i+1), i);
    end
    ocp_solver.set('x', ocp_solver.get('x',N_horizon), N_horizon);
    ocp_solver.set('u', ocp_solver.get('u',N_horizon-1), N_horizon-1);

    x = sim_solver.simulate(x, u);
    if mod(k,20)==0 || k<=5
        fprintf('Step %3d: z=%.4f mm  |u|=%.3f\n', k, x(3)*1e3, norm(u));
    end
    if abs(x(3))>0.5 || any(isnan(x))
        fprintf('DIVERGED at step %d\n', k); break;
    end
end
fprintf('Final: z=%.4f mm, |pos err|=%.4f mm\n', x(3)*1e3, norm(x(1:3)-xEq(1:3))*1e3);
