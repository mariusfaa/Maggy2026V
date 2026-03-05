%% validate_dynamics.m
% Step 6: Validate CasADi maglevSystemDynamics_casadi vs MATLAB
%
% Compares state derivatives at equilibrium and perturbed states.
%
% Pass criteria: max relative error < 2%

% clear; clc;
addpath(genpath('..'));

import casadi.*

%% Setup
load_params(MaglevModel.Accurate);

luts = buildLuts(params);
params.luts = luts;

%% Build CasADi function
x_sym = MX.sym('x', 12, 1);
u_sym = MX.sym('u', 4, 1);

dx_sym = maglevSystemDynamics_casadi(x_sym, u_sym, params);
dyn_func = Function('dyn', {x_sym, u_sym}, {dx_sym});

%% Test states
z_eq = 0.042;

test_cases = {
    % {state, currents, description}
    {[0; 0; z_eq; 0; 0; 0; 0; 0; 0; 0; 0; 0], [0;0;0;0], 'Equilibrium, u=0'};
    {[0; 0; z_eq; 0; 0; 0; 0; 0; 0; 0; 0; 0], [1;0;0;0], 'Equilibrium, u=[1,0,0,0]'};
    {[0.005; 0; z_eq; 0; 0; 0; 0; 0; 0; 0; 0; 0], [0;0;0;0], 'x-offset'};
    {[0; 0; z_eq+0.005; 0; 0; 0; 0; 0; 0; 0; 0; 0], [0;0;0;0], 'z-offset'};
    {[0; 0; z_eq; 0.05; 0; 0; 0; 0; 0; 0; 0; 0], [0;0;0;0], 'Roll tilt'};
    {[0; 0; z_eq; 0; 0; 0; 0.1; 0; 0; 0; 0; 0], [0;0;0;0], 'With velocity'};
    {[0; 0; z_eq; 0; 0; 0; 0; 0; 0; 0; 0; 1], [0;0;0;0], 'With spin'};
    {[0.003; 0.003; z_eq; 0.02; 0.02; 0; 0; 0; 0; 0; 0; 0], [1;-1;1;-1], 'Combined'};
};

fprintf('\n=== Step 6: CasADi Dynamics vs MATLAB ===\n');
max_rel_err = 0;

for k = 1:length(test_cases)
    x0 = test_cases{k}{1};
    u0 = test_cases{k}{2};
    desc = test_cases{k}{3};

    % CasADi
    dx_casadi = full(dyn_func(x0, u0));

    % MATLAB reference
    dx_ref = maglevSystemDynamics(x0, u0, params, MaglevModel.Accurate);

    % Mixed error metric
    eps_dx = max(abs(dx_ref)) * 0.01;
    mixed_err = abs(dx_casadi - dx_ref) ./ (abs(dx_ref) + eps_dx);
    max_err_k = max(mixed_err);
    max_rel_err = max(max_rel_err, max_err_k);

    fprintf('%20s: max_rel_err = %.4f%%\n', desc, max_err_k*100);
    fprintf('  dx_casadi = [%.4e, %.4e, %.4e, ... %.4e, %.4e, %.4e]\n', ...
        dx_casadi(1), dx_casadi(2), dx_casadi(3), ...
        dx_casadi(10), dx_casadi(11), dx_casadi(12));
    fprintf('  dx_ref    = [%.4e, %.4e, %.4e, ... %.4e, %.4e, %.4e]\n', ...
        dx_ref(1), dx_ref(2), dx_ref(3), ...
        dx_ref(10), dx_ref(11), dx_ref(12));
end

fprintf('\nOverall max relative error: %.4f%%\n', max_rel_err*100);
if max_rel_err < 0.02
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end
