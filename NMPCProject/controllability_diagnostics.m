%% Controllability Diagnostics - Why is rank = 6 instead of 10?
% This script helps debug why your controllability matrix is rank deficient

clear; close all;

fprintf('=== CONTROLLABILITY DIAGNOSTICS ===\n\n');

%% Setup
project_root = '/home/mariujf/Maggy2026V/NMPCProject';
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% Load Parameters
fprintf('1. Loading parameters...\n');

try
    parameters_maggy_V2;
    fprintf('   Using V2 parameters\n');
    using_v2 = true;
catch
    parameters_maggy_V4;
    fprintf('   Using V4 parameters (V2 not found)\n');
    using_v2 = false;
end

correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

%% Compute Equilibrium
fprintf('\n2. Computing equilibrium...\n');

[zEq, ~, ~, ~] = computeSystemEquilibria(paramsFast,'fast');
xEq = [0, 0, zEq(1), zeros(1,9)]';
uEq = zeros(length(params.solenoids.r), 1);

fprintf('   Equilibrium: z = %.2f mm\n', zEq(1)*1000);

%% Linearization Method 1: Original (Numerical)
fprintf('\n3. Linearization Method 1: Numerical (Original)\n');

modelName = 'fast';
f_original = @(x,u) maglevSystemDynamics(x, u, paramsFast, modelName);
h_original = @(x,u) maglevSystemMeasurements(x, u, paramsFast, modelName);

delta = 1e-6;
[A_num, B_num, C_num, D_num] = finiteDifferenceLinearization(f_original, h_original, xEq, uEq, delta);

fprintf('   A: %dx%d, B: %dx%d\n', size(A_num), size(B_num));

% Reduced system
I = [1:5, 7:11];
Ared_num = A_num(I,I);
Bred_num = B_num(I,:);

% Controllability
ctrb_num = ctrb(Ared_num, Bred_num);
rank_num = rank(ctrb_num);
rank_num_tol = rank(ctrb_num, 1e-6);  % With tolerance

fprintf('   Controllability rank: %d (tol=eps), %d (tol=1e-6)\n', rank_num, rank_num_tol);
fprintf('   Expected: 10\n');

if rank_num == 10
    fprintf('   ✓ NUMERICAL METHOD WORKS\n');
else
    fprintf('   ✗ NUMERICAL METHOD FAILS - this shouldn''t happen!\n');
end

%% Linearization Method 2: CasADi Symbolic
fprintf('\n4. Linearization Method 2: CasADi Symbolic (Your Method)\n');

nx = 12;
nu = length(params.solenoids.r);

x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);

% Use YOUR CasADi dynamics
f_casadi = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);

% Compute Jacobians
jac_A = jacobian(f_casadi, x_sym);
jac_B = jacobian(f_casadi, u_sym);

% Create functions
A_fun = Function('A_fun', {x_sym, u_sym}, {jac_A});
B_fun = Function('B_fun', {x_sym, u_sym}, {jac_B});

% Evaluate
A_sym = full(A_fun(xEq, uEq));
B_sym = full(B_fun(xEq, uEq));

fprintf('   A: %dx%d, B: %dx%d\n', size(A_sym), size(B_sym));

% Reduced system
Ared_sym = A_sym(I,I);
Bred_sym = B_sym(I,:);

% Controllability
ctrb_sym = ctrb(Ared_sym, Bred_sym);
rank_sym = rank(ctrb_sym);
rank_sym_tol = rank(ctrb_sym, 1e-6);

fprintf('   Controllability rank: %d (tol=eps), %d (tol=1e-6)\n', rank_sym, rank_sym_tol);
fprintf('   Expected: 10\n');

if rank_sym == 10
    fprintf('   ✓ CASADI METHOD WORKS\n');
else
    fprintf('   ✗ CASADI METHOD FAILS - rank = %d\n', rank_sym);
end

%% Compare A and B Matrices
fprintf('\n5. Comparing A and B matrices...\n');

A_diff = A_num - A_sym;
B_diff = B_num - B_sym;

fprintf('   ||A_num - A_sym|| = %.6e\n', norm(A_diff, 'fro'));
fprintf('   ||B_num - B_sym|| = %.6e\n', norm(B_diff, 'fro'));
fprintf('   Relative error (A): %.3f%%\n', 100*norm(A_diff,'fro')/norm(A_num,'fro'));
fprintf('   Relative error (B): %.3f%%\n', 100*norm(B_diff,'fro')/norm(B_num,'fro'));

if norm(A_diff, 'fro') > 1e-4 || norm(B_diff, 'fro') > 1e-4
    fprintf('   ⚠️  LARGE DIFFERENCE - Dynamics implementations differ!\n');
else
    fprintf('   ✓ Matrices are close\n');
end

%% Compare Dynamics Functions Directly
fprintf('\n6. Comparing dynamics functions at equilibrium...\n');

dx_num = f_original(xEq, uEq);
dx_sym = full(maglevSystemDynamicsCasADi(xEq, uEq, paramsFast));

fprintf('   ||f_num(xEq,uEq)|| = %.6e\n', norm(dx_num));
fprintf('   ||f_sym(xEq,uEq)|| = %.6e\n', norm(dx_sym));
fprintf('   Difference: %.6e\n', norm(dx_num - dx_sym));

if norm(dx_num - dx_sym) > 1e-8
    fprintf('   ⚠️  Dynamics differ at equilibrium!\n');
    fprintf('      This could indicate wrong equilibrium or wrong dynamics\n');
else
    fprintf('   ✓ Both give near-zero at equilibrium (good)\n');
end

%% Test at Perturbed Point
fprintf('\n7. Comparing dynamics at perturbed point...\n');

x_test = xEq + [0.001, 0.002, 0.005, 0.01, -0.01, 0, zeros(1,6)]';
u_test = zeros(nu, 1);

dx_num_test = f_original(x_test, u_test);
dx_sym_test = full(maglevSystemDynamicsCasADi(x_test, u_test, paramsFast));

diff = dx_num_test - dx_sym_test;

fprintf('   Test point: x = xEq + [1mm, 2mm, 5mm, 0.01rad, ...]\n');
fprintf('   ||f_num - f_sym|| = %.6e\n', norm(diff));
fprintf('   Relative error: %.3f%%\n', 100*norm(diff)/norm(dx_num_test));

fprintf('\n   State-by-state comparison:\n');
fprintf('   State   |  Numerical  |   CasADi    | Difference \n');
fprintf('   --------|-------------|-------------|------------\n');
for i = 1:nx
    fprintf('   x(%2d)   | %+11.6f | %+11.6f | %+10.3e\n', ...
        i, dx_num_test(i), dx_sym_test(i), diff(i));
end

if norm(diff) > 1e-6
    fprintf('\n   ✗ DYNAMICS DIFFER SIGNIFICANTLY\n');
    fprintf('     This is the root cause of controllability issue!\n');
else
    fprintf('\n   ✓ Dynamics match\n');
end

%% Analyze Uncontrollable Modes
fprintf('\n8. Analyzing uncontrollable subspace...\n');

if rank_sym < 10
    % Compute null space of controllability matrix
    null_ctrb = null(ctrb_sym);
    
    fprintf('   Dimension of uncontrollable subspace: %d\n', size(null_ctrb, 2));
    fprintf('   Uncontrollable mode vectors:\n');
    
    for i = 1:size(null_ctrb, 2)
        fprintf('\n   Mode %d:\n', i);
        fprintf('   States [x y z roll pitch vx vy vz wx wy]:\n');
        fprintf('   [');
        for j = 1:10
            fprintf('%.3f ', null_ctrb(j, i));
        end
        fprintf(']\n');
    end
    
    % Check eigenvalues of uncontrollable modes
    fprintf('\n   Eigenvalues of A_red:\n');
    eigs_A = eig(Ared_sym);
    for i = 1:length(eigs_A)
        if abs(imag(eigs_A(i))) < 1e-10
            fprintf('   λ%d = %.4f (real)\n', i, real(eigs_A(i)));
        else
            fprintf('   λ%d = %.4f %+.4fi\n', i, real(eigs_A(i)), imag(eigs_A(i)));
        end
    end
end

%% Check B Matrix Structure
fprintf('\n9. Analyzing B matrix structure...\n');

fprintf('   B_num (numerical):\n');
fprintf('   First 3 rows (position derivatives - should be zero):\n');
disp(B_num(1:3, :));

fprintf('\n   Rows 4-6 (orientation derivatives - should be zero):\n');
disp(B_num(4:6, :));

fprintf('\n   Rows 7-9 (velocity derivatives - should be nonzero):\n');
disp(B_num(7:9, :));

fprintf('\n   B_sym (CasADi):\n');
fprintf('   Rows 7-9 (velocity derivatives):\n');
disp(B_sym(7:9, :));

fprintf('\n   Difference in critical rows:\n');
disp(B_num(7:9, :) - B_sym(7:9, :));

%% Summary
fprintf('\n=== SUMMARY ===\n');
fprintf('Numerical method rank: %d\n', rank_num);
fprintf('CasADi method rank: %d\n', rank_sym);

if rank_num == 10 && rank_sym < 10
    fprintf('\n✗ PROBLEM: CasADi dynamics are wrong!\n');
    fprintf('  Action: Fix maglevSystemDynamicsCasADi.m\n');
    fprintf('  Likely issue: Euler angle kinematics (dx(4:6) = omega is wrong)\n');
elseif rank_num < 10 && rank_sym < 10
    fprintf('\n✗ PROBLEM: Both methods fail!\n');
    fprintf('  Action: Check equilibrium point and system parameters\n');
elseif rank_num == 10 && rank_sym == 10
    fprintf('\n✓ BOTH METHODS WORK!\n');
    fprintf('  Controller should be stable\n');
else
    fprintf('\n? UNEXPECTED: Numerical fails but CasADi works?\n');
end

if norm(A_diff, 'fro') > 1e-4
    fprintf('\nA matrices differ by %.2e - dynamics are NOT equivalent!\n', norm(A_diff, 'fro'));
end

if norm(B_diff, 'fro') > 1e-4
    fprintf('B matrices differ by %.2e - control coupling is wrong!\n', norm(B_diff, 'fro'));
end

fprintf('\n=== END DIAGNOSTICS ===\n');
