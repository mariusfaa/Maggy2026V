%% Emergency Diagnostics - NaN/Inf and Rank Issue
% Both methods fail - this points to a fundamental issue

clear; close all;

fprintf('=== EMERGENCY DIAGNOSTICS ===\n\n');

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
catch
    parameters_maggy_V4;
    fprintf('   Using V4 parameters\n');
end

correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

fprintf('   params.magnet.n = %d\n', paramsFast.magnet.n);

%% Compute Equilibrium
fprintf('\n2. Computing equilibrium...\n');

[zEq, ~, ~, ~] = computeSystemEquilibria(paramsFast,'fast');
xEq = [0, 0, zEq(1), zeros(1,9)]';
uEq = zeros(length(params.solenoids.r), 1);

fprintf('   Equilibrium: z = %.4f m (%.2f mm)\n', zEq(1), zEq(1)*1000);

% CRITICAL: Verify this is actually an equilibrium
modelName = 'fast';
f_original = @(x,u) maglevSystemDynamics(x, u, paramsFast, modelName);
dx_at_eq = f_original(xEq, uEq);

fprintf('   Checking equilibrium: ||f(xEq,uEq)|| = %.6e\n', norm(dx_at_eq));
if norm(dx_at_eq) > 1e-6
    fprintf('   ⚠️  WARNING: This is NOT a true equilibrium!\n');
    fprintf('   State derivatives at "equilibrium":\n');
    for i = 1:12
        fprintf('      dx(%2d) = %+.6e\n', i, dx_at_eq(i));
    end
end

%% Check for NaN/Inf in Dynamics at Equilibrium
fprintf('\n3. Checking for NaN/Inf in dynamics...\n');

nx = 12;
nu = length(params.solenoids.r);

% Test with original function
fprintf('   Testing original function at equilibrium:\n');
dx_orig = f_original(xEq, uEq);
if any(isnan(dx_orig)) || any(isinf(dx_orig))
    fprintf('   ✗ ORIGINAL FUNCTION PRODUCES NaN/Inf!\n');
    for i = 1:12
        if isnan(dx_orig(i)) || isinf(dx_orig(i))
            fprintf('      dx(%2d) = %s\n', i, num2str(dx_orig(i)));
        end
    end
else
    fprintf('   ✓ Original function OK\n');
end

% Test with CasADi function
fprintf('\n   Testing CasADi function at equilibrium:\n');
try
    dx_casadi = full(maglevSystemDynamicsCasADi(xEq, uEq, paramsFast));
    if any(isnan(dx_casadi)) || any(isinf(dx_casadi))
        fprintf('   ✗ CASADI FUNCTION PRODUCES NaN/Inf!\n');
        for i = 1:12
            if isnan(dx_casadi(i)) || isinf(dx_casadi(i))
                fprintf('      dx(%2d) = %s\n', i, num2str(dx_casadi(i)));
            end
        end
    else
        fprintf('   ✓ CasADi function OK\n');
    end
catch ME
    fprintf('   ✗ CASADI FUNCTION CRASHED: %s\n', ME.message);
end

%% Check Jacobians for NaN/Inf
fprintf('\n4. Computing Jacobians and checking for NaN/Inf...\n');

x_sym = SX.sym('x', nx);
u_sym = SX.sym('u', nu);

try
    f_casadi = maglevSystemDynamicsCasADi(x_sym, u_sym, paramsFast);
    fprintf('   ✓ Symbolic expression created\n');
    
    % Compute Jacobians
    jac_A = jacobian(f_casadi, x_sym);
    jac_B = jacobian(f_casadi, u_sym);
    fprintf('   ✓ Jacobians computed symbolically\n');
    
    % Create functions
    A_fun = Function('A_fun', {x_sym, u_sym}, {jac_A});
    B_fun = Function('B_fun', {x_sym, u_sym}, {jac_B});
    fprintf('   ✓ Functions created\n');
    
    % Evaluate at equilibrium
    fprintf('\n   Evaluating Jacobians at equilibrium...\n');
    A_eval = full(A_fun(xEq, uEq));
    B_eval = full(B_fun(xEq, uEq));
    
    % Check for NaN/Inf
    A_has_nan = any(isnan(A_eval(:)));
    A_has_inf = any(isinf(A_eval(:)));
    B_has_nan = any(isnan(B_eval(:)));
    B_has_inf = any(isinf(B_eval(:)));
    
    fprintf('   A matrix: %d NaN, %d Inf\n', sum(isnan(A_eval(:))), sum(isinf(A_eval(:))));
    fprintf('   B matrix: %d NaN, %d Inf\n', sum(isnan(B_eval(:))), sum(isinf(B_eval(:))));
    
    if A_has_nan || A_has_inf
        fprintf('\n   ⚠️  A MATRIX HAS NaN/Inf!\n');
        fprintf('   Showing problematic entries:\n');
        [row, col] = find(isnan(A_eval) | isinf(A_eval));
        for i = 1:min(10, length(row))
            fprintf('      A(%d,%d) = %s\n', row(i), col(i), num2str(A_eval(row(i), col(i))));
        end
    end
    
    if B_has_nan || B_has_inf
        fprintf('\n   ⚠️  B MATRIX HAS NaN/Inf!\n');
        fprintf('   Showing problematic entries:\n');
        [row, col] = find(isnan(B_eval) | isinf(B_eval));
        for i = 1:min(10, length(row))
            fprintf('      B(%d,%d) = %s\n', row(i), col(i), num2str(B_eval(row(i), col(i))));
        end
    end
    
catch ME
    fprintf('   ✗ ERROR during Jacobian evaluation:\n');
    fprintf('      %s\n', ME.message);
    fprintf('      Stack:\n');
    for i = 1:length(ME.stack)
        fprintf('        %s (line %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
end

%% Check Equilibrium Point More Carefully
fprintf('\n5. Analyzing equilibrium point...\n');

fprintf('   Full equilibrium state:\n');
fprintf('   x    = [%+.4f %+.4f %+.4f] m\n', xEq(1:3));
fprintf('   ang  = [%+.4f %+.4f %+.4f] rad\n', xEq(4:6));
fprintf('   vel  = [%+.4f %+.4f %+.4f] m/s\n', xEq(7:9));
fprintf('   avel = [%+.4f %+.4f %+.4f] rad/s\n', xEq(10:12));

% Check for special values
if xEq(3) < 0.001
    fprintf('   ⚠️  Equilibrium height is very low (%.2f mm)!\n', xEq(3)*1000);
end

if xEq(3) > 0.100
    fprintf('   ⚠️  Equilibrium height is very high (%.2f mm)!\n', xEq(3)*1000);
end

% Check if angles are at singularity
if abs(abs(xEq(5)) - pi/2) < 0.01  % pitch near ±90°
    fprintf('   ✗ PITCH ANGLE NEAR SINGULARITY: %.2f deg\n', rad2deg(xEq(5)));
    fprintf('      Euler angle kinematics are singular here!\n');
end

%% Test Perturbed Points
fprintf('\n6. Testing dynamics at perturbed points...\n');

perturbations = [
    0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;     % Small x
    0, 0, 0.002, 0, 0, 0, 0, 0, 0, 0, 0, 0;     % Small z
    0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0;      % Small roll
];

for i = 1:size(perturbations, 1)
    x_test = xEq + perturbations(i, :)';
    
    try
        dx_test = full(maglevSystemDynamicsCasADi(x_test, uEq, paramsFast));
        has_nan = any(isnan(dx_test)) || any(isinf(dx_test));
        
        if has_nan
            fprintf('   Perturbation %d: ✗ NaN/Inf\n', i);
        else
            fprintf('   Perturbation %d: ✓ OK (norm = %.3e)\n', i, norm(dx_test));
        end
    catch ME
        fprintf('   Perturbation %d: ✗ ERROR: %s\n', i, ME.message);
    end
end

%% Specific Issue: Check params.magnet.n
fprintf('\n7. Checking discretization parameter...\n');

fprintf('   params.magnet.n = %d\n', paramsFast.magnet.n);

if paramsFast.magnet.n < 3
    fprintf('   ✗ n is too small! Need at least n=3 for trapezoidal rule\n');
elseif paramsFast.magnet.n < 10
    fprintf('   ⚠️  n is very small - may cause numerical issues\n');
else
    fprintf('   ✓ n seems reasonable\n');
end

%% The Real Question: Why is Numerical Rank = 6?
fprintf('\n8. Investigating why numerical linearization gives rank=6...\n');

fprintf('   This suggests the ACTUAL SYSTEM has uncontrollable modes,\n');
fprintf('   not a bug in your code!\n\n');

fprintf('   Possible reasons:\n');
fprintf('   1. Wrong equilibrium point (not a true equilibrium)\n');
fprintf('   2. Structural uncontrollability with n=%d discretization\n', paramsFast.magnet.n);
fprintf('   3. Actuator configuration cannot control all modes\n');
fprintf('   4. Parameters V4 differ from V2 in fundamental ways\n\n');

%% Check Original sim_notlive.m Settings
fprintf('9. Comparing with working original...\n\n');

fprintf('   CRITICAL QUESTIONS:\n');
fprintf('   Q1: Are you using the SAME parameter file as sim_notlive.m?\n');
fprintf('       - Original uses: parameters_maggy_V2\n');
fprintf('       - You are using: parameters_maggy_V%s\n', ...
    replace(func2str(@parameters_maggy_V4), 'parameters_maggy_V', ''));
fprintf('\n   Q2: What is params.magnet.n in the working original?\n');
fprintf('       - Yours: %d\n', paramsFast.magnet.n);
fprintf('       - Original: ??? (check parameters_maggy_V2.m)\n');
fprintf('\n   Q3: Did you apply the correction factor correctly?\n');
fprintf('       - Correction factor: %.4f\n', correctionFactorFast);
fprintf('       - Original radius: %.4f mm\n', params.solenoids.r(1)*1000);
fprintf('       - Corrected radius: %.4f mm\n', paramsFast.solenoids.r(1)*1000);

%% ACTION ITEMS
fprintf('\n=== ACTION ITEMS ===\n\n');

fprintf('1. VERIFY PARAMETER FILE\n');
fprintf('   Run the original sim_notlive.m and check what it prints for:\n');
fprintf('   - Equilibrium height\n');
fprintf('   - Controllability rank\n');
fprintf('   - params.magnet.n\n\n');

fprintf('2. CHECK IF NaN/Inf IN JACOBIANS\n');
fprintf('   If A or B matrices have NaN/Inf, the issue is:\n');
fprintf('   - Singularity in Euler angle transform at theta=±90°\n');
fprintf('   - Division by zero in magnetic field calculation\n');
fprintf('   - Invalid elliptic integral parameters\n\n');

fprintf('3. IF NUMERICAL RANK = 6 IN ORIGINAL TOO\n');
fprintf('   Then the system IS actually rank-deficient!\n');
fprintf('   Check if original sim_notlive.m prints rank=6 or rank=10\n');
fprintf('   Maybe the tolerance difference (eps vs 1e-6) matters\n\n');

fprintf('4. COMPARE EXACT NUMBERS\n');
fprintf('   Run sim_notlive.m and copy the exact output here:\n');
fprintf('   - Equilibrium: z = ???\n');
fprintf('   - Correction factor: ???\n');
fprintf('   - Controllability rank: ???\n');

fprintf('\n=== END DIAGNOSTICS ===\n');
