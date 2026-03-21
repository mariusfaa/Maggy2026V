%% Observability Analysis: Matrix vs. Gramian
clear; close all; clc;

%% 1. Create a system that's "barely observable"
% This system will pass the rank test but is practically unobservable
% System matrices
A = Ad;
C = Cred;

n = size(A, 1); % Number of states

%% 2. Traditional Observability Matrix Check
fprintf('=== Traditional Observability Matrix Analysis ===\n');

% Build observability matrix
O = obsv(A, C);
rank_O = rank(O);
fprintf('Observability matrix rank: %d (system dimension: %d)\n', rank_O, n);

if rank_O == n
    fprintf('✓ System is OBSERVABLE by rank test\n');
else
    fprintf('✗ System is UNOBSERVABLE by rank test\n');
end

% Check numerical conditioning
cond_O = cond(O);
fprintf('Condition number of O: %.2e\n', cond_O);
fprintf('   (Large condition number indicates numerical issues)\n\n');

%% 3. Observability Gramian Analysis
fprintf('=== Observability Gramian Analysis ===\n');

% Check stability first (Gramian requires stable A)
eigA = eig(A);
if all(real(eigA) < 0)
    % System is stable, solve continuous-time Lyapunov equation
    % For discrete systems: use dlyap(A', C'*C)
    W_O = lyap(A', C'*C);
else
    % For unstable or discrete systems, use the integral definition
    % We'll simulate over a finite time horizon
    fprintf('System not strictly stable, using finite-time Gramian\n');
    T = 0.005;  % Time horizon
    t = 0:dt:T;
    W_O = zeros(n);
    
    % Compute Discrete Gramian
    for k = 0:10%length(t)-1
        W_O = W_O + (A')^k * (C'*C) * A^k;
    end
end

%% 4. Analyze the Gramian
fprintf('\nObservability Gramian W_O:\n');
disp(W_O);

% Eigenvalue analysis (shows degree of observability)
[eigvecs, eigvals] = eig(W_O);
eigvals_diag = diag(eigvals);
[eigvals_sorted, idx] = sort(eigvals_diag, 'descend');
eigvecs_sorted = eigvecs(:, idx);

fprintf('\nEigenvalues of W_O (sorted):\n');
for i = 1:n
    fprintf('  λ%d = %.4e', i, eigvals_sorted(i));
    if eigvals_sorted(i) < 1e-6
        fprintf('  → PRACTICALLY UNOBSERVABLE\n');
    elseif eigvals_sorted(i) < 1e-3
        fprintf('  → WEAKLY OBSERVABLE\n');
    else
        fprintf('  → STRONGLY OBSERVABLE\n');
    end
end

% Condition number of Gramian
cond_WO = cond(W_O);
fprintf('\nCondition number of W_O: %.2e\n', cond_WO);

% Smallest eigenvalue (most critical)
min_eig = min(eigvals_diag);
fprintf('Smallest eigenvalue: %.4e\n', min_eig);

% Determinant (volume of observability ellipsoid)
det_WO = det(W_O);
fprintf('Determinant of W_O: %.4e\n', det_WO);

%% 5. Compare state observability
fprintf('\n=== State Observability Comparison ===\n');
fprintf('Energy required to observe initial states (x0''*W_O*x0):\n\n');

% Test with different initial conditions
test_states = eye(n);  % Unit vectors for each state

for i = 1:n
    x0 = test_states(:, i);
    output_energy = x0' * W_O * x0;
    fprintf('Initial state x%d = [%s]: Output Energy = %.4e', ...
            i, num2str(x0'), output_energy);
    
    if output_energy < 1e-6
        fprintf('  → State %d is practically unobservable!\n', i);
    elseif output_energy < 1e-3
        fprintf('  → State %d is weakly observable\n', i);
    else
        fprintf('  → State %d is strongly observable\n', i);
    end
end

% Check eigenvectors (directions of observability)
fprintf('\nMost observable direction (principal eigenvector):\n');
fprintf('  Direction: [%s]\n', num2str(eigvecs_sorted(:,1)', '%.3f  '));
fprintf('  Corresponding eigenvalue: %.4e\n', eigvals_sorted(1));

fprintf('\nLeast observable direction:\n');
fprintf('  Direction: [%s]\n', num2str(eigvecs_sorted(:,end)', '%.3f  '));
fprintf('  Corresponding eigenvalue: %.4e\n', eigvals_sorted(end));

