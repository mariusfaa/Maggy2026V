%% Direct Comparison: Original vs Your Setup
% Run this to see EXACTLY what differs

clear; close all;

fprintf('=== DIRECT COMPARISON: Original sim_notlive.m vs Your Setup ===\n\n');

%% Setup
project_root = '/home/mariujf/Maggy2026V/NMPCProject';
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

import casadi.*

%% PART A: Exactly as Original sim_notlive.m
fprintf('═══════════════════════════════════════\n');
fprintf('PART A: ORIGINAL sim_notlive.m method\n');
fprintf('═══════════════════════════════════════\n\n');

try
    parameters_maggy_V2;
    fprintf('✓ Using parameters_maggy_V2 (original)\n');
    params_A = params;
catch
    fprintf('✗ parameters_maggy_V2 not found!\n');
    fprintf('  This could be the root cause!\n\n');
    fprintf('  Trying V4 instead...\n');
    parameters_maggy_V4;
    params_A = params;
end

% Apply correction
correctionFactorFast_A = computeSolenoidRadiusCorrectionFactor(params_A,'fast');
paramsFast_A = params_A;
paramsFast_A.solenoids.r = correctionFactorFast_A * paramsFast_A.solenoids.r;

fprintf('  Correction factor: %.4f\n', correctionFactorFast_A);
fprintf('  params.magnet.n: %d\n', paramsFast_A.magnet.n);

% Equilibrium
[zEq_A, ~, ~, ~] = computeSystemEquilibria(paramsFast_A,'fast');
xLp_A = [0, 0, zEq_A(1), zeros(1,9)]';
uLp_A = zeros(length(params_A.solenoids.r), 1);

fprintf('  Equilibrium z: %.4f m (%.2f mm)\n', zEq_A(1), zEq_A(1)*1000);

% Linearization - EXACTLY AS ORIGINAL
modelName = 'fast';
f_A = @(x,u) maglevSystemDynamics(x, u, paramsFast_A, modelName);
h_A = @(x,u) maglevSystemMeasurements(x, u, paramsFast_A, modelName);

delta = 1e-6;
fprintf('  Computing linearization (delta=%.0e)...\n', delta);
[A_A, B_A, C_A, D_A] = finiteDifferenceLinearization(f_A, h_A, xLp_A, uLp_A, delta);

% Controllability - EXACTLY AS ORIGINAL
I = [1:5, 7:11];
Ared_A = A_A(I,I);
Bred_A = B_A(I,:);

ctrb_A = ctrb(Ared_A, Bred_A);
rank_A_eps = rank(ctrb_A);  % Default tolerance (eps)
rank_A_1e6 = rank(ctrb_A, 1e-6);  % Loose tolerance

fprintf('\n  RESULTS (Original Method):\n');
fprintf('  ─────────────────────────────\n');
fprintf('  Controllability rank (tol=eps):  %d\n', rank_A_eps);
fprintf('  Controllability rank (tol=1e-6): %d\n', rank_A_1e6);
fprintf('  Expected: 10\n\n');

if rank_A_1e6 == 10
    fprintf('  ✓ System is controllable (with tolerance 1e-6)\n');
else
    fprintf('  ✗ System is NOT fully controllable\n');
    fprintf('    Missing %d controllable modes\n', 10 - rank_A_1e6);
end

% Check singular values to see how close
sv_A = svd(ctrb_A);
fprintf('\n  Singular values of controllability matrix:\n');
for i = 1:min(10, length(sv_A))
    fprintf('    σ%d = %.3e', i, sv_A(i));
    if sv_A(i) < 1e-6
        fprintf(' ← Near zero!');
    end
    fprintf('\n');
end

%% PART B: Your CasADi Method
fprintf('\n═══════════════════════════════════════\n');
fprintf('PART B: YOUR CasADi method\n');
fprintf('═══════════════════════════════════════\n\n');

% Use same parameters as Part A for fair comparison
paramsFast_B = paramsFast_A;
xLp_B = xLp_A;
uLp_B = uLp_A;

fprintf('  Using SAME parameters as Part A for comparison\n');
fprintf('  params.magnet.n: %d\n', paramsFast_B.magnet.n);

nx = 12;
nu = length(params_A.solenoids.r);

x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);

fprintf('  Computing symbolic Jacobians...\n');

try
    % Get dynamics
    f_B = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast_B);
    
    % Jacobians
    jac_A_B = jacobian(f_B, x_sym);
    jac_B_B = jacobian(f_B, u_sym);
    
    % Functions
    A_fun_B = Function('A_fun', {x_sym, u_sym}, {jac_A_B});
    B_fun_B = Function('B_fun', {x_sym, u_sym}, {jac_B_B});
    
    % Evaluate
    fprintf('  Evaluating at equilibrium...\n');
    A_B = full(A_fun_B(xLp_B, uLp_B));
    B_B = full(B_fun_B(xLp_B, uLp_B));
    
    % Check for NaN/Inf
    if any(isnan(A_B(:))) || any(isinf(A_B(:)))
        fprintf('\n  ✗ ERROR: A matrix contains NaN/Inf!\n');
        fprintf('    NaN count: %d\n', sum(isnan(A_B(:))));
        fprintf('    Inf count: %d\n', sum(isinf(A_B(:))));
        
        % Find problematic entries
        [row, col] = find(isnan(A_B) | isinf(A_B));
        fprintf('\n    Problematic entries in A:\n');
        for i = 1:min(5, length(row))
            fprintf('      A(%d,%d) = %s\n', row(i), col(i), num2str(A_B(row(i), col(i))));
        end
        
        error('Cannot continue - A matrix has NaN/Inf');
    end
    
    if any(isnan(B_B(:))) || any(isinf(B_B(:)))
        fprintf('\n  ✗ ERROR: B matrix contains NaN/Inf!\n');
        fprintf('    NaN count: %d\n', sum(isnan(B_B(:))));
        fprintf('    Inf count: %d\n', sum(isinf(B_B(:))));
        
        error('Cannot continue - B matrix has NaN/Inf');
    end
    
    fprintf('  ✓ Matrices computed successfully (no NaN/Inf)\n');
    
    % Controllability
    Ared_B = A_B(I,I);
    Bred_B = B_B(I,:);
    
    ctrb_B = ctrb(Ared_B, Bred_B);
    rank_B_eps = rank(ctrb_B);
    rank_B_1e6 = rank(ctrb_B, 1e-6);
    
    fprintf('\n  RESULTS (CasADi Method):\n');
    fprintf('  ─────────────────────────────\n');
    fprintf('  Controllability rank (tol=eps):  %d\n', rank_B_eps);
    fprintf('  Controllability rank (tol=1e-6): %d\n', rank_B_1e6);
    fprintf('  Expected: 10\n\n');
    
    if rank_B_1e6 == 10
        fprintf('  ✓ System is controllable (with tolerance 1e-6)\n');
    else
        fprintf('  ✗ System is NOT fully controllable\n');
        fprintf('    Missing %d controllable modes\n', 10 - rank_B_1e6);
    end
    
    % Singular values
    sv_B = svd(ctrb_B);
    fprintf('\n  Singular values of controllability matrix:\n');
    for i = 1:min(10, length(sv_B))
        fprintf('    σ%d = %.3e', i, sv_B(i));
        if sv_B(i) < 1e-6
            fprintf(' ← Near zero!');
        end
        fprintf('\n');
    end
    
    %% PART C: Comparison
    fprintf('\n═══════════════════════════════════════\n');
    fprintf('PART C: COMPARISON\n');
    fprintf('═══════════════════════════════════════\n\n');
    
    fprintf('Matrix differences:\n');
    fprintf('  ||A_orig - A_casi|| = %.3e\n', norm(A_A - A_B, 'fro'));
    fprintf('  ||B_orig - B_casi|| = %.3e\n', norm(B_A - B_B, 'fro'));
    
    if norm(A_A - A_B, 'fro') < 1e-4
        fprintf('  ✓ A matrices match\n');
    else
        fprintf('  ✗ A matrices differ significantly\n');
    end
    
    if norm(B_A - B_B, 'fro') < 1e-4
        fprintf('  ✓ B matrices match\n');
    else
        fprintf('  ✗ B matrices differ significantly\n');
    end
    
    fprintf('\nControllability comparison:\n');
    fprintf('  Original rank (1e-6): %d\n', rank_A_1e6);
    fprintf('  CasADi rank (1e-6):   %d\n', rank_B_1e6);
    
    if rank_A_1e6 == rank_B_1e6
        fprintf('  ✓ Same rank!\n');
        if rank_A_1e6 < 10
            fprintf('\n  BOTH methods give rank %d < 10\n', rank_A_1e6);
            fprintf('  This means the SYSTEM ITSELF is uncontrollable,\n');
            fprintf('  not a bug in your CasADi implementation!\n');
        end
    else
        fprintf('  ✗ Different ranks!\n');
    end
    
catch ME
    fprintf('\n✗ ERROR in CasADi method:\n');
    fprintf('  %s\n', ME.message);
    fprintf('\nStack trace:\n');
    for i = 1:length(ME.stack)
        fprintf('  %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

%% PART D: Root Cause Analysis
fprintf('\n═══════════════════════════════════════\n');
fprintf('PART D: ROOT CAUSE ANALYSIS\n');
fprintf('═══════════════════════════════════════\n\n');

if rank_A_1e6 < 10
    fprintf('The original numerical method ALSO gives rank < 10!\n\n');
    fprintf('This means:\n');
    fprintf('  • The problem is NOT your CasADi implementation\n');
    fprintf('  • The problem is NOT the Euler angle fix\n');
    fprintf('  • The system with these parameters is inherently uncontrollable\n\n');
    
    fprintf('Possible reasons:\n');
    fprintf('  1. params.magnet.n = %d is too low\n', paramsFast_A.magnet.n);
    fprintf('     → Try increasing to 20, 50, or 100\n\n');
    
    fprintf('  2. Wrong parameter file\n');
    fprintf('     → Original uses V2, you might be using V4\n');
    fprintf('     → Check if V2 gives different results\n\n');
    
    fprintf('  3. Tolerance issue\n');
    fprintf('     → rank=6 with eps, rank=10 with 1e-6\n');
    fprintf('     → System is "barely" controllable\n');
    fprintf('     → Small σ values indicate near-uncontrollability\n\n');
    
    fprintf('  4. Equilibrium point issue\n');
    fprintf('     → Check if equilibrium is correct\n');
    fprintf('     → Try different equilibrium heights\n\n');
    
    fprintf('NEXT STEP:\n');
    fprintf('  Run the ORIGINAL sim_notlive.m and check:\n');
    fprintf('  • What rank does IT print?\n');
    fprintf('  • What is params.magnet.n in V2?\n');
    fprintf('  • Does it work despite rank < 10?\n');
else
    fprintf('Both methods give rank = 10 with tolerance 1e-6\n');
    fprintf('The system IS controllable!\n');
    fprintf('Controller should work.\n');
end

fprintf('\n=== END COMPARISON ===\n');
