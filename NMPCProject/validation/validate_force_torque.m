%% validate_force_torque.m
% Step 5: Validate CasADi computeForceAndTorque_casadi vs MATLAB
%
% Compares force and torque at various states and current values against
% the MATLAB computeForceAndTorque (Accurate model).
%
% Pass criteria: max relative error < 2% on forces, < 5% on torques

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

[fx_s, fy_s, fz_s, tx_s, ty_s, tz_s] = computeForceAndTorque_casadi(x_sym, u_sym, params);

ft_func = Function('ft', {x_sym, u_sym}, {fx_s, fy_s, fz_s, tx_s, ty_s, tz_s});

%% Test states
% Equilibrium height ~42mm for this system
z_eq = 0.042;

test_states = {
    [0; 0; z_eq; 0; 0; 0; 0; 0; 0; 0; 0; 0],                         % Equilibrium
    [0.0025; 0; z_eq; 0; 0; 0; 0; 0; 0; 0; 0; 0],                     % x offset
    [0; 0; z_eq+0.004; 0; 0; 0; 0; 0; 0; 0; 0; 0],                   % z offset
    [0; 0; z_eq; 0.05; 0; 0; 0; 0; 0; 0; 0; 0],                      % roll tilt
    [0; 0; z_eq; 0; 0.05; 0; 0; 0; 0; 0; 0; 0],                      % pitch tilt
    [0.0013; 0.0023; z_eq; 0.02; 0.02; 0; 0; 0; 0; 0; 0; 0],           % combined
};

test_currents = {
    [0; 0; 0; 0],
    [1; 0; 0; 0],
    [1; -1; 1; -1],
    [0.5; 0.5; -0.5; -0.5],
};

fprintf('\n=== Step 5: CasADi Force/Torque vs MATLAB (Accurate) ===\n');

%% First pass: collect all reference values to compute global eps
all_force_ref = [];
all_torque_ref = [];

for s = 1:length(test_states)
    for c = 1:length(test_currents)
        [fx_r, fy_r, fz_r, tx_r, ty_r, tz_r] = computeForceAndTorque( ...
            test_states{s}, test_currents{c}, params, MaglevModel.Accurate);
        all_force_ref = [all_force_ref; fx_r, fy_r, fz_r]; %#ok<AGROW>
        all_torque_ref = [all_torque_ref; tx_r, ty_r, tz_r]; %#ok<AGROW>
    end
end

eps_f_global = max(abs(all_force_ref(:))) * 0.01;
eps_t_global = max(abs(all_torque_ref(:))) * 0.01;

%% Second pass: evaluate and compare
max_force_err = 0;
max_torque_err = 0;

for s = 1:length(test_states)
    for c = 1:length(test_currents)
        x0 = test_states{s};
        u0 = test_currents{c};

        % CasADi evaluation
        [fx_c, fy_c, fz_c, tx_c, ty_c, tz_c] = ft_func(x0, u0);
        ft_casadi = [full(fx_c), full(fy_c), full(fz_c), full(tx_c), full(ty_c), full(tz_c)];

        % MATLAB reference (reuse from first pass)
        idx = (s-1)*length(test_currents) + c;
        ft_ref = [all_force_ref(idx,:), all_torque_ref(idx,:)];

        % Force errors (mixed metric with global eps)
        force_cas = ft_casadi(1:3);
        force_ref = ft_ref(1:3);
        rel_f = abs(force_cas - force_ref) ./ (abs(force_ref) + eps_f_global);
        max_force_err = max(max_force_err, max(rel_f));

        % Torque errors (mixed metric with global eps)
        torque_cas = ft_casadi(4:6);
        torque_ref = ft_ref(4:6);
        rel_t = abs(torque_cas - torque_ref) ./ (abs(torque_ref) + eps_t_global);
        max_torque_err = max(max_torque_err, max(rel_t));

        fprintf('State %d, Current %d:\n', s, c);
        fprintf('  F: cas=[%.4e, %.4e, %.4e] ref=[%.4e, %.4e, %.4e] maxerr=%.2f%%\n', ...
            ft_casadi(1:3), ft_ref(1:3), max(rel_f)*100);
        fprintf('  T: cas=[%.4e, %.4e, %.4e] ref=[%.4e, %.4e, %.4e] maxerr=%.2f%%\n', ...
            ft_casadi(4:6), ft_ref(4:6), max(rel_t)*100);
    end
end

fprintf('\nMax force  relative error: %.4f%%\n', max_force_err*100);
fprintf('Max torque relative error: %.4f%%\n', max_torque_err*100);
fprintf('Global eps: force=%.2e, torque=%.2e\n', eps_f_global, eps_t_global);

if max_force_err < 0.02 && max_torque_err < 0.05
    fprintf('PASS\n');
else
    fprintf('FAIL\n');
end
