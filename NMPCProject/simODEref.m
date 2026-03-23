%% simODEref — Reference simulation using MATLAB ode15s

simSetup;

modelId = MaglevModel.Accurate;
params = load_params(modelId);  % reload with correct solenoid correction
save_filename = sprintf('ode_acc_n%d_na%d.mat',params.magnet.n,params.magnet.n_axial);
fprintf("%s\n",save_filename);

%% --- System equations ---
f_ode = @(x, u) maglevSystemDynamics(x, u, params, modelId);

%% --- Simulate ---
fprintf('--- Running ode15s reference simulation ---\n');
fprintf('  x0 = [%.4f, %.4f, %.4f, %.4f, %.4f, ...]\n', x0(1:5));
fprintf('  u0 = [%.4f, %.4f, %.4f, %.4f]\n', u0);
fprintf('  T  = %.3f s,  dt = %.1f us\n', t(end), dt*1e6);

ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);
t = [0, t(end)];

tic;
[t_out, x_out] = ode15s(@(t, x) f_ode(x, u0), t, x0, ode_opts);
t_elapsed = toc;
x_out = x_out';

fprintf('ode15s done in %.3f s (%d output points)\n', t_elapsed, size(x_out, 2));

%% --- SAVE ---
sim_data      = struct();
sim_data.t    = t_out(:)';
sim_data.x    = x_out;
sim_data.u    = repmat(u0, 1, size(x_out, 2));
sim_data.xEq  = xEq;
sim_data.uEq  = uEq;
sim_data.dt   = dt;

save(save_filename, '-struct', 'sim_data');
fprintf('ODE results saved to: %s\n', save_filename);
