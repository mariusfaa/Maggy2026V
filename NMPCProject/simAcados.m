%% simAcados — Open-loop acados integrator simulation (Fast model)

simSetup;
import casadi.*

%clear sim_solver;
save_filename = 'results_acados_reduced3.mat';

%% --- Model setup ---
fprintf('--- Setting up model ---\n');

if ~exist("model","var")
    model = get_maggy_model(MaglevModel.Fast,use_luts=false);
end

%% --- BUILD / REUSE SIM SOLVER ---
fprintf('\n--- Building acados sim solver ---\n');

if ~exist("sim_solver","var")
    sim = AcadosSim();
    sim.model = model;

    sim.solver_options.Tsim            = dt;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 1;

    sim_solver = AcadosSimSolver(sim);
end

%% --- Run simulation ---
x_traj    = zeros(nx, numel(t));
u_traj    = zeros(nu, numel(t));
t_tot = zeros(1, numel(t));
t_la = zeros(1, numel(t));
t_ad = zeros(1, numel(t));


u = u0;
x = x0;

for i = 1:numel(t)
    x_traj(:, i) = x;
    u_traj(:, i) = u;

    x = sim_solver.simulate(x, u);
    t_tot(i) = sim_solver.get('time_tot');
    t_la(i) = sim_solver.get('time_la');
    t_ad(i) = sim_solver.get('time_ad');

    diverged = abs(x(3)) > 0.5       || ...
               max(abs(x(4:5))) > pi  || ...
               any(isnan(x))          || ...
               any(isinf(x));

    fprintf('Step %4d: |x|=%.4e  simtime=%.0f us\n', ...
        i, norm(x), t_tot(i)*1e6);

    if diverged
        fprintf('  *** DIVERGED at step %d ***\n', i);
        x_traj = x_traj(:, 1:i);
        u_traj = u_traj(:, 1:i);
        t = t(1:i);
        t_tot = t_tot(1:i);
        t_la = t_la(1:i);
        t_ad = t_ad(1:i);
        break;
    end
end

%% --- Performance summary ---
fprintf('\n--- Sim Performance ---\n');
fprintf('Per step t_tot: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_tot)*1e6, max(t_tot)*1e6, median(t_tot)*1e6);
fprintf('Per step t_la: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_la)*1e6, max(t_la)*1e6, median(t_la)*1e6);
fprintf('Per step t_ad: mean=%.0f us, max=%.0f us, median=%.0f us\n', ...
    mean(t_ad)*1e6, max(t_ad)*1e6, median(t_ad)*1e6);

%% --- SAVE ---
sim_data      = struct();
sim_data.t    = t;
sim_data.x    = x_traj;
sim_data.u    = u_traj;
sim_data.xEq  = xEq;
sim_data.uEq  = uEq;
sim_data.dt   = dt;

sim_data.t_tot  = t_tot;
sim_data.t_la  = t_la;
sim_data.t_ad  = t_ad;

save(save_filename, '-struct', 'sim_data');
fprintf('\nSimulation data saved to: %s\n', save_filename);