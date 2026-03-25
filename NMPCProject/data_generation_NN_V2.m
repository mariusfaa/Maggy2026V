%% generate_nn_data_v3.m
% =========================================================================
%  Generate training data for the LINEARITY DECOMPOSITION approach.
%
%  KEY INSIGHT: The magnetic force/torque is LINEAR in solenoid currents.
%  Therefore the acceleration decomposes as:
%
%    accel(pose, u) = accel_perm(pose) + Σᵢ uᵢ · accel_solᵢ(pose)
%
%  We train TWO networks (both with 6 inputs = pose only):
%    NN_perm: pose → 6 outputs  (permanent magnet acceleration at u=0)
%    NN_sol:  pose → 24 outputs (4 solenoids × 6 accel per unit amp)
%
%  ADVANTAGES over the single-network approach:
%  1. Input dimension drops from 10 to 6 (much easier for small MLP)
%  2. Linearity in current is EXACT by construction
%  3. ∂f/∂u = NN_sol(pose) — no chain rule through NN for control Jacobian
%  4. Each NN sees a smoother function (no u-pose interaction)
%
%  DATA GENERATION: At each pose, evaluate dynamics 5 times:
%    u = [0,0,0,0]  → accel_perm
%    u = [1,0,0,0]  → accel_perm + accel_sol1  →  accel_sol1 = diff
%    u = [0,1,0,0]  → accel_perm + accel_sol2  →  accel_sol2 = diff
%    u = [0,0,1,0]  → accel_perm + accel_sol3  →  accel_sol3 = diff
%    u = [0,0,0,1]  → accel_perm + accel_sol4  →  accel_sol4 = diff
%
%  OUTPUT: nn_training_data_v3.mat
% =========================================================================

clearvars; clc;

%% --- PROJECT SETUP ---
acados_root  = '/home/mariujf/acados';
project_root = '/home/mariujf/Maggy2026V/NMPCProject';

addpath(fullfile(acados_root, 'external', 'casadi-matlab'));
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% --- BUILD CASADI FUNCTION ---
fprintf('=== Building CasADi dynamics function ===\n');

nx = 12;  nu = 4;
x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);

parameters_maggy_V4;
correctionFactorFast   = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast             = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

f_expl  = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);
f_func  = casadi.Function('f', {x_sym, u_sym}, {f_expl});

%% --- FIND EQUILIBRIUM ---
fprintf('=== Finding equilibrium ===\n');

z_var    = SX.sym('z_eq');
x_eq_sym = [0; 0; z_var; zeros(9,1)];
accel    = f_func(x_eq_sym, zeros(nu,1));

nlp       = struct('x', z_var, 'f', accel(9)^2);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 0)));

sol     = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
zEq_cas = full(sol.x);
uEq     = zeros(nu, 1);
xEq     = [0; 0; zEq_cas; zeros(9,1)];

fprintf('  Equilibrium: z = %.6f m\n', zEq_cas);

%% --- VERIFY LINEARITY ---
fprintf('\n=== Verifying linearity in u ===\n');

% Test at a random pose
test_pose = xEq;
test_pose(1) = 0.005; test_pose(4) = 0.05; test_pose(5) = -0.03;

u_test = [0.3; -0.5; 0.7; -0.2];

% Direct evaluation
dx_direct = full(f_func(test_pose, u_test));
accel_direct = dx_direct(7:12);

% Decomposed evaluation
dx_u0 = full(f_func(test_pose, [0;0;0;0]));
accel_u0 = dx_u0(7:12);

accel_decomp = accel_u0;
for i = 1:4
    e_i = zeros(4,1); e_i(i) = 1;
    dx_ei = full(f_func(test_pose, e_i));
    accel_sol_i = dx_ei(7:12) - accel_u0;
    accel_decomp = accel_decomp + u_test(i) * accel_sol_i;
end

linearity_error = norm(accel_direct - accel_decomp);
fprintf('  Direct:      [%.4e, %.4e, ...]\n', accel_direct(1), accel_direct(2));
fprintf('  Decomposed:  [%.4e, %.4e, ...]\n', accel_decomp(1), accel_decomp(2));
fprintf('  Linearity error: %.4e  (should be ~machine precision)\n', linearity_error);

if linearity_error > 1e-6
    error('Linearity assumption violated! Error = %.4e', linearity_error);
end
fprintf('  CONFIRMED: acceleration is linear in u.\n');

%% --- DEFINE SAMPLING DOMAIN ---
% Same narrowed domain as v2
ranges = struct();
ranges.x_min     = -0.025;    ranges.x_max     =  0.025;
ranges.y_min     = -0.025;    ranges.y_max     =  0.025;
ranges.z_min     =  0.020;    ranges.z_max     =  0.045;
ranges.roll_min  = -0.25;     ranges.roll_max  =  0.25;
ranges.pitch_min = -0.25;     ranges.pitch_max =  0.25;
ranges.yaw_min   = -pi;       ranges.yaw_max   =  pi;

%% --- GENERATE POSES ---
N_near_eq = 300000;
N_medium  = 100000;
N_uniform = 100000;
N_total   = N_near_eq + N_medium + N_uniform;

fprintf('\n=== Generating %d poses ===\n', N_total);

rng(42);

% Tier 1: Near-equilibrium
eq_std_1 = [0.003; 0.003; 0.003; 0.03; 0.03; 0.2];
p1 = xEq(1:6) + eq_std_1 .* randn(6, N_near_eq);

% Tier 2: Medium spread
eq_std_2 = [0.010; 0.010; 0.008; 0.10; 0.10; 0.5];
p2 = xEq(1:6) + eq_std_2 .* randn(6, N_medium);

% Tier 3: Uniform
p3 = [ranges.x_min     + (ranges.x_max     - ranges.x_min)     * rand(1, N_uniform);
      ranges.y_min     + (ranges.y_max     - ranges.y_min)     * rand(1, N_uniform);
      ranges.z_min     + (ranges.z_max     - ranges.z_min)     * rand(1, N_uniform);
      ranges.roll_min  + (ranges.roll_max  - ranges.roll_min)  * rand(1, N_uniform);
      ranges.pitch_min + (ranges.pitch_max - ranges.pitch_min) * rand(1, N_uniform);
      ranges.yaw_min   + (ranges.yaw_max   - ranges.yaw_min)   * rand(1, N_uniform)];

all_pose = [p1, p2, p3];

% Clamp
all_pose(1,:) = max(min(all_pose(1,:), ranges.x_max),     ranges.x_min);
all_pose(2,:) = max(min(all_pose(2,:), ranges.y_max),     ranges.y_min);
all_pose(3,:) = max(min(all_pose(3,:), ranges.z_max),     ranges.z_min);
all_pose(4,:) = max(min(all_pose(4,:), ranges.roll_max),  ranges.roll_min);
all_pose(5,:) = max(min(all_pose(5,:), ranges.pitch_max), ranges.pitch_min);
all_pose(6,:) = max(min(all_pose(6,:), ranges.yaw_max),   ranges.yaw_min);

%% --- EVALUATE DYNAMICS (5 evaluations per pose) ---
fprintf('=== Evaluating dynamics (%d poses × 5 evals = %d total) ===\n', ...
    N_total, N_total*5);

nn_pose   = all_pose;           % 6 x N
nn_perm   = zeros(6, N_total);  % permanent magnet accel
nn_sol    = zeros(24, N_total); % solenoid accel (4 solenoids × 6 components)

x_full = zeros(12, 1);
e_vecs = eye(4);  % unit vectors for each solenoid

tic;
print_interval = 50000;
for i = 1:N_total
    x_full(1:6)  = all_pose(:, i);
    x_full(7:12) = 0;

    % Evaluate at u = 0 (permanent magnet contribution)
    dx_u0 = full(f_func(x_full, [0;0;0;0]));
    accel_u0 = dx_u0(7:12);
    nn_perm(:, i) = accel_u0;

    % Evaluate at each unit current to get per-solenoid contribution
    for j = 1:4
        dx_ej = full(f_func(x_full, e_vecs(:,j)));
        accel_sol_j = dx_ej(7:12) - accel_u0;
        nn_sol((j-1)*6+1 : j*6, i) = accel_sol_j;
    end

    if mod(i, print_interval) == 0
        elapsed = toc;
        rate = i / elapsed;
        eta = (N_total - i) / rate;
        fprintf('  %d / %d  (%.0f poses/s, ETA: %.0f s)\n', i, N_total, rate, eta);
    end
end
elapsed = toc;
fprintf('  Done in %.1f s  (%.0f poses/s, %d total evals)\n', ...
    elapsed, N_total/elapsed, N_total*5);

%% --- COMPUTE EQUILIBRIUM TARGETS ---
fprintf('\n=== Computing equilibrium targets ===\n');

x_eq_full = xEq;
dx_eq_u0 = full(f_func(x_eq_full, [0;0;0;0]));
eq_perm_target = dx_eq_u0(7:12);
fprintf('  accel_perm at eq (should be ~0): [%.4e, %.4e, %.4e, %.4e, %.4e, %.4e]\n', ...
    eq_perm_target);

eq_sol_target = zeros(24, 1);
for j = 1:4
    dx_ej = full(f_func(x_eq_full, e_vecs(:,j)));
    eq_sol_target((j-1)*6+1 : j*6) = dx_ej(7:12) - eq_perm_target;
end
fprintf('  accel_sol at eq (solenoid sensitivities):\n');
for j = 1:4
    idx = (j-1)*6+1 : j*6;
    fprintf('    Sol %d: lin=[%.2e, %.2e, %.2e], ang=[%.2e, %.2e, %.2e]\n', ...
        j, eq_sol_target(idx(1)), eq_sol_target(idx(2)), eq_sol_target(idx(3)), ...
        eq_sol_target(idx(4)), eq_sol_target(idx(5)), eq_sol_target(idx(6)));
end

%% --- CLIP NN_PERM OUTPUTS ---
CLIP_PERM_ANG = 50000;
n_clipped = sum(any(abs(nn_perm(4:6,:)) > CLIP_PERM_ANG, 1));
fprintf('\n=== Clipping NN_perm angular at +/-%d (%.1f%% of samples) ===\n', ...
    CLIP_PERM_ANG, 100*n_clipped/N_total);
nn_perm(4:6,:) = max(min(nn_perm(4:6,:), CLIP_PERM_ANG), -CLIP_PERM_ANG);

%% --- STATISTICS ---
fprintf('\n=== Output statistics ===\n');

fprintf('  NN_perm (permanent magnet accel):\n');
perm_names = {'ax_perm', 'ay_perm', 'az_perm', 'alphax_perm', 'alphay_perm', 'alphaz_perm'};
for j = 1:6
    fprintf('    %12s: range=[%+.4e, %+.4e], std=%.4e\n', ...
        perm_names{j}, min(nn_perm(j,:)), max(nn_perm(j,:)), std(nn_perm(j,:)));
end

fprintf('  NN_sol (per-solenoid accel per amp) — showing angular channels:\n');
for s = 1:4
    idx = (s-1)*6 + (4:6);
    for k = 1:3
        ch = idx(k);
        fprintf('    sol%d_alpha%d: range=[%+.4e, %+.4e], std=%.4e\n', ...
            s, k, min(nn_sol(ch,:)), max(nn_sol(ch,:)), std(nn_sol(ch,:)));
    end
end

%% --- SAVE ---
output_file = fullfile(project_root, 'nn_training_data_v3.mat');

save(output_file, 'nn_pose', 'nn_perm', 'nn_sol', ...
     'xEq', 'uEq', 'eq_perm_target', 'eq_sol_target', 'paramsFast');

fprintf('\n=== Saved to %s ===\n', output_file);
fprintf('  nn_pose: %d x %d  (6D pose inputs)\n', size(nn_pose));
fprintf('  nn_perm: %d x %d  (permanent magnet accel)\n', size(nn_perm));
fprintf('  nn_sol:  %d x %d  (solenoid accel, 4×6)\n', size(nn_sol));
fprintf('\n  NEXT: python train_surrogate_v3.py\n');