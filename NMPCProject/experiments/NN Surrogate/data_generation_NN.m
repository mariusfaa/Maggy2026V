%% generate_nn_data_v2.m
% =========================================================================
%  Generate training data for the NN surrogate model — VERSION 2.
%
%  CHANGES FROM V1 (and why):
%  ──────────────────────────────────────────────────────────────────────
%  1. NARROWER SAMPLING DOMAIN:
%     V1 sampled z ∈ [12, 58]mm, roll/pitch ∈ [-0.40, 0.40] rad.
%     At the extremes (z=12mm + 23° tilt + 1A current), angular
%     accelerations reach 2×10⁸ rad/s². The MLP_3x64 spends all its
%     capacity fitting these physically unreachable outliers.
%
%     V2 uses z ∈ [20, 45]mm, roll/pitch ∈ [-0.25, 0.25] rad.
%     This still covers the full state constraint region with margin,
%     but avoids the extreme field gradients near the base.
%
%  2. OUTPUT CLIPPING:
%     Even within the narrower domain, some configurations produce
%     extreme accelerations. We clip outputs to bound the dynamic
%     range the NN must learn.
%
%  3. MORE NEAR-EQUILIBRIUM SAMPLES:
%     60% of samples clustered near equilibrium (vs 20% in V1),
%     because that's where the NMPC spends 99% of its time.
%
%  OUTPUT FILE: nn_training_data.mat
%    nn_input  : 10 x N   [x y z roll pitch yaw u1 u2 u3 u4]
%    nn_output :  6 x N   [ax ay az alpha_x alpha_y alpha_z]
%    xEq       : 12 x 1
%    uEq       :  4 x 1
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
x = SX.sym('x', nx);
u = SX.sym('u', nu);

parameters_maggy_V4;
correctionFactorFast   = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast             = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

f_expl  = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_func  = casadi.Function('f', {x, u}, {f_expl});

%% --- FIND EQUILIBRIUM ---
fprintf('=== Finding equilibrium ===\n');

z_var    = SX.sym('z_eq');
u_zero   = zeros(nu, 1);
x_eq_sym = [0; 0; z_var; zeros(9,1)];
accel    = f_func(x_eq_sym, u_zero);
fz_res   = accel(9);

nlp       = struct('x', z_var, 'f', fz_res^2);
solver_eq = nlpsol('solver_eq', 'ipopt', nlp, ...
    struct('ipopt', struct('print_level', 0)));

sol     = solver_eq('x0', 0.030, 'lbx', 0.015, 'ubx', 0.060);
zEq_cas = full(sol.x);
uEq     = zeros(nu, 1);
xEq     = [0; 0; zEq_cas; zeros(9,1)];

fprintf('  Equilibrium: z = %.6f m\n', zEq_cas);
dx_eq = full(f_func(xEq, uEq));
fprintf('  Residual: %.4e\n', norm(dx_eq));

%% --- DEFINE SAMPLING RANGES ---
% NARROWER than V1 — avoids extreme field gradients
%
% Rationale for each range:
%   x,y: ±25mm is the state constraint — keep as-is (these don't cause
%         extreme accelerations)
%   z:   State constraint is [15,55]mm, but z<20mm puts the magnet
%         extremely close to the base permanent magnets where field
%         gradients are astronomical. Narrowing to [20,45]mm still
%         covers the useful operating region.
%   roll/pitch: State constraint is ±0.35 rad = ±20°. At 20° tilt with
%         low z, torques are extreme. Narrowing to ±0.25 rad (±14°)
%         avoids the worst cases while covering the operating region.
%   yaw:  Full range [-pi, pi] — yaw doesn't cause extreme torques.
%   u:    Full range [-1, 1] A — the controller can use full authority.

ranges = struct();
ranges.x_min     = -0.025;    ranges.x_max     =  0.025;
ranges.y_min     = -0.025;    ranges.y_max     =  0.025;
ranges.z_min     =  0.020;    ranges.z_max     =  0.045;    % was [0.012, 0.058]
ranges.roll_min  = -0.25;     ranges.roll_max  =  0.25;     % was [-0.40, 0.40]
ranges.pitch_min = -0.25;     ranges.pitch_max =  0.25;     % was [-0.40, 0.40]
ranges.yaw_min   = -pi;       ranges.yaw_max   =  pi;
ranges.u_min     = -1.0;      ranges.u_max     =  1.0;

% Output clipping thresholds
% These are generous: at equilibrium accel≈0, in normal operation
% linear accel ~ tens of m/s², angular accel ~ thousands of rad/s².
% Clipping at these values removes only the unphysical extremes.
CLIP_LIN  = 500;     % m/s²    (~50g — well beyond anything the NMPC sees)
CLIP_ANG  = 50000;  % rad/s²  (generous; normal operation is ~1000s)

%% --- GENERATE SAMPLES ---
N_near_eq = 300000;   % 60% near equilibrium
N_medium  = 100000;   % 20% moderate spread
N_uniform = 100000;   % 20% full domain
N_total   = N_near_eq + N_medium + N_uniform;

fprintf('\n=== Generating %d training samples ===\n', N_total);
fprintf('  %d near-eq + %d medium + %d uniform\n', N_near_eq, N_medium, N_uniform);
fprintf('  Domain: z=[%.0f,%.0f]mm, roll/pitch=[%.2f,%.2f]rad\n', ...
    ranges.z_min*1e3, ranges.z_max*1e3, ranges.roll_min, ranges.roll_max);

rng(42);

% ---- Tier 1: Near-equilibrium (very tight Gaussian) ----
eq_std_1 = [0.003; 0.003; 0.003; 0.03; 0.03; 0.2];  % pose
eq_std_u1 = [0.2; 0.2; 0.2; 0.2];                     % current
p1 = xEq(1:6) + eq_std_1 .* randn(6, N_near_eq);
u1 = uEq       + eq_std_u1 .* randn(4, N_near_eq);

% ---- Tier 2: Medium spread (wider Gaussian) ----
eq_std_2 = [0.010; 0.010; 0.008; 0.10; 0.10; 0.5];
eq_std_u2 = [0.5; 0.5; 0.5; 0.5];
p2 = xEq(1:6) + eq_std_2 .* randn(6, N_medium);
u2 = uEq       + eq_std_u2 .* randn(4, N_medium);

% ---- Tier 3: Uniform across the domain ----
p3 = [ranges.x_min     + (ranges.x_max     - ranges.x_min)     * rand(1, N_uniform);
      ranges.y_min     + (ranges.y_max     - ranges.y_min)     * rand(1, N_uniform);
      ranges.z_min     + (ranges.z_max     - ranges.z_min)     * rand(1, N_uniform);
      ranges.roll_min  + (ranges.roll_max  - ranges.roll_min)  * rand(1, N_uniform);
      ranges.pitch_min + (ranges.pitch_max - ranges.pitch_min) * rand(1, N_uniform);
      ranges.yaw_min   + (ranges.yaw_max   - ranges.yaw_min)   * rand(1, N_uniform)];
u3 = ranges.u_min + (ranges.u_max - ranges.u_min) * rand(4, N_uniform);

% ---- Combine and clamp to valid ranges ----
all_pose = [p1, p2, p3];
all_u    = [u1, u2, u3];

all_pose(1,:) = max(min(all_pose(1,:), ranges.x_max),     ranges.x_min);
all_pose(2,:) = max(min(all_pose(2,:), ranges.y_max),     ranges.y_min);
all_pose(3,:) = max(min(all_pose(3,:), ranges.z_max),     ranges.z_min);
all_pose(4,:) = max(min(all_pose(4,:), ranges.roll_max),  ranges.roll_min);
all_pose(5,:) = max(min(all_pose(5,:), ranges.pitch_max), ranges.pitch_min);
all_pose(6,:) = max(min(all_pose(6,:), ranges.yaw_max),   ranges.yaw_min);
all_u         = max(min(all_u,         ranges.u_max),      ranges.u_min);

nn_input = [all_pose; all_u];  % 10 x N_total

%% --- EVALUATE DYNAMICS ---
fprintf('=== Evaluating dynamics at %d points ===\n', N_total);

nn_output = zeros(6, N_total);
x_full = zeros(12, 1);

tic;
print_interval = 50000;
for i = 1:N_total
    x_full(1:6)  = nn_input(1:6, i);
    x_full(7:12) = 0;

    u_i = nn_input(7:10, i);
    dx  = full(f_func(x_full, u_i));

    nn_output(:, i) = dx(7:12);

    if mod(i, print_interval) == 0
        elapsed = toc;
        rate = i / elapsed;
        eta = (N_total - i) / rate;
        fprintf('  %d / %d  (%.0f samples/s, ETA: %.0f s)\n', i, N_total, rate, eta);
    end
end
elapsed = toc;
fprintf('  Done in %.1f s  (%.0f samples/s)\n', elapsed, N_total/elapsed);

%% --- CLIP OUTPUTS ---
fprintf('\n=== Clipping extreme outputs ===\n');

n_clipped_lin = sum(any(abs(nn_output(1:3,:)) > CLIP_LIN, 1));
n_clipped_ang = sum(any(abs(nn_output(4:6,:)) > CLIP_ANG, 1));
fprintf('  Samples with |linear accel| > %.0f:  %d (%.2f%%)\n', ...
    CLIP_LIN, n_clipped_lin, 100*n_clipped_lin/N_total);
fprintf('  Samples with |angular accel| > %.0f: %d (%.2f%%)\n', ...
    CLIP_ANG, n_clipped_ang, 100*n_clipped_ang/N_total);

nn_output(1:3,:) = max(min(nn_output(1:3,:),  CLIP_LIN), -CLIP_LIN);
nn_output(4:6,:) = max(min(nn_output(4:6,:),  CLIP_ANG), -CLIP_ANG);

%% --- VALIDATION ---
fprintf('\n=== Validation ===\n');

fprintf('  Output statistics (AFTER clipping):\n');
names = {'ax', 'ay', 'az', 'alpha_x', 'alpha_y', 'alpha_z'};
for j = 1:6
    fprintf('    %8s: min=%+.4e, max=%+.4e, mean=%+.4e, std=%.4e\n', ...
        names{j}, min(nn_output(j,:)), max(nn_output(j,:)), ...
        mean(nn_output(j,:)), std(nn_output(j,:)));
end

% NaN/Inf check
n_nan = sum(any(isnan(nn_output), 1));
n_inf = sum(any(isinf(nn_output), 1));
fprintf('\n  NaN: %d,  Inf: %d\n', n_nan, n_inf);
if n_nan > 0 || n_inf > 0
    valid = ~any(isnan(nn_output), 1) & ~any(isinf(nn_output), 1);
    nn_input  = nn_input(:, valid);
    nn_output = nn_output(:, valid);
    N_total = size(nn_input, 2);
end

%% --- SAVE ---
output_file = fullfile(project_root, 'nn_training_data.mat');
save(output_file, 'nn_input', 'nn_output', 'xEq', 'uEq', 'paramsFast');
fprintf('\n=== Saved %d samples to %s ===\n', N_total, output_file);
fprintf('  nn_input:  %d x %d\n', size(nn_input));
fprintf('  nn_output: %d x %d\n', size(nn_output));