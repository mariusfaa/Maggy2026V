%% simAcados_dipole — Open-loop acados simulation (Dipole model)
%
% Runs the same scenario as simODEref_dipole but using the acados ERK
% integrator with the CasADi dipole model. Compare results to validate
% the CasADi implementation against the MATLAB reference.

simSetup;
import casadi.*

nx_red = 10;

save_filename = 'results_acados_dipole.mat';

%% --- Dipole equilibrium and initial conditions ---
params_dip = load_params(MaglevModel.Dipole);
[zEq_dip, ~, ~, ~] = computeSystemEquilibria(params_dip, MaglevModel.Dipole);
xEq_red = [0; 0; zEq_dip(1); zeros(7,1)];
uEq = zeros(nu, 1);

% Same perturbation as simODEref_dipole
x0_full = [0; 0; zEq_dip(1); zeros(9,1)] + [0; 0.001; 0.001; 0; 0; 0; zeros(6,1)];
x0_red = x0_full([1:5,7:11]);
u0 = [-0.25; 0.5; -0.5; 0.75];

fprintf('Dipole equilibrium: z = %.4f mm\n', zEq_dip(1)*1e3);

%% --- Build model ---
model = get_maggy_model(MaglevModel.Dipole);

%% --- Sim solver ---
sim = AcadosSim();
sim.model = model;

sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;
sim.solver_options.ext_fun_compile_flags = '-O2';

fprintf('\n--- Building acados sim solver (Dipole) ---\n');
sim_solver = AcadosSimSolver(sim);

%% --- Run simulation ---
N_sim = numel(t);
x_sim = zeros(nx_red, N_sim);
u_sim = zeros(nu, N_sim);
t_sim_log = zeros(1, N_sim);

x_cur = x0_red;

fprintf('\n--- Running acados simulation (Dipole, open-loop) ---\n');
fprintf('  T=%.3fs, dt=%.1f us, N=%d\n', t(end), dt*1e6, N_sim);
fprintf('  u0 = [%.4f, %.4f, %.4f, %.4f]\n', u0);

diverged = false;

for i = 1:N_sim
    x_sim(:, i) = x_cur;
    u_sim(:, i) = u0;

    x_cur = sim_solver.simulate(x_cur, u0);
    t_sim_log(i) = sim_solver.get('time_tot');

    diverged = abs(x_cur(3)) > 0.5       || ...
               max(abs(x_cur(4:5))) > pi  || ...
               any(isnan(x_cur))          || ...
               any(isinf(x_cur));

    if diverged
        fprintf('  DIVERGED at step %d (t=%.4f s)\n', i, t(i));
        x_sim = x_sim(:, 1:i);
        u_sim = u_sim(:, 1:i);
        t = t(1:i);
        t_sim_log = t_sim_log(1:i);
        break;
    end
end

if ~diverged
    fprintf('Simulation completed without divergence.\n');
end

fprintf('Sim step: mean=%.0f us, median=%.0f us, max=%.0f us\n', ...
    mean(t_sim_log)*1e6, median(t_sim_log)*1e6, max(t_sim_log)*1e6);
fprintf('Final z = %.4f mm\n', x_cur(3)*1e3);

%% --- SAVE ---
sim_data         = struct();
sim_data.t       = t;
sim_data.x       = x_sim;
sim_data.u       = u_sim;
sim_data.xEq     = xEq_red;
sim_data.uEq     = uEq;
sim_data.dt      = dt;
sim_data.t_tot   = t_sim_log;

save(save_filename, '-struct', 'sim_data');
fprintf('Results saved to: %s\n', save_filename);
