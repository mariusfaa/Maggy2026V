%% --- ODE Reference Simulation ---
% Simulates the maglev system using MATLAB's ode15s (stiff solver) with
% the original maglevSystemDynamics. Uses the same initial conditions and
% inputs as simAcados so results can be directly compared.

simSetup;

% Setting up system equations
f = @(x,u) maglevSystemDynamics(x,u,params,modelId);
h = @(x,u) maglevSystemMeasurements(x,u,params,modelId);

%% --- Simulate with ode15s ---
fprintf('--- Running ode15s reference simulation ---\n');
fprintf('  x0 = [%.4f, %.4f, %.4f, %.4f, %.4f, ...]\n', x0(1:5));
fprintf('  u0 = [%.4f, %.4f, %.4f, %.4f]\n', u0);
fprintf('  T  = %.3f s\n', t(end));

ode_opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-9);

tic;
[t_out, x_out] = ode15s(@(t,x) f(x, u0), t, x0, ode_opts);
t_elapsed = toc;
x_out = x_out';

fprintf('ode15s done in %.3f s (%d output points)\n', t_elapsed, size(x_out,2));

%% --- Save ---
save_filename = 'results_ode.mat';
ode_data        = struct();
ode_data.t      = t_out(:)';
ode_data.x      = x_out;
ode_data.u      = repmat(u0, 1, size(x_out,2));
ode_data.xEq    = xEq;
ode_data.uEq    = uEq;
ode_data.dt     = dt;

save(save_filename, '-struct', 'ode_data');
fprintf('ODE results saved to: %s\n', save_filename);
