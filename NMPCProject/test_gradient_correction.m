%% test_gradient_correction — Verify Jacobian matching after gradient correction
%
% Compares the linearized dynamics of:
%   1. Accurate model at its equilibrium
%   2. Dipole model (with gradient correction) at its equilibrium
%
% If the gradient correction works, the A matrices should be very close.

addpath(genpath('model_matlab'));
addpath(genpath('system_parameters'));
addpath(genpath('utilities'));

fprintf('=== Gradient Correction Verification ===\n\n');

%% Load both models
params_acc = load_params(MaglevModel.Accurate);
params_dip = load_params(MaglevModel.Dipole);  % includes gradient correction

[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);
[zEq_dip, ~, ~, ~] = computeSystemEquilibria(params_dip, MaglevModel.Dipole);

fprintf('\nAccurate eq: z = %.4f mm\n', zEq_acc(1)*1e3);
fprintf('Dipole eq:   z = %.4f mm\n', zEq_dip(1)*1e3);

%% Compute 10-state Jacobians via finite differences
% 10-state: [x,y,z,roll,pitch,vx,vy,vz,wx,wy]
% Map to 12-state for MATLAB force computation

nx = 10; nu = 4;
m = params_acc.magnet.m;
I_vec = params_acc.magnet.I;
g = params_acc.physical.g;

% Dynamics function for 10-state model
f_acc = @(x10, u) dynamics10(x10, u, params_acc, MaglevModel.Accurate, m, I_vec, g);

% Dipole dynamics WITH gradient correction
f_dip = @(x10, u) dynamics10_corrected(x10, u, params_dip, m, I_vec, g);

% Dipole dynamics WITHOUT gradient correction (for comparison)
params_dip_nocorr = params_dip;
params_dip_nocorr = rmfield(params_dip_nocorr, 'dipole_corr');
f_dip_nocorr = @(x10, u) dynamics10(x10, u, params_dip_nocorr, MaglevModel.Dipole, m, I_vec, g);

% Equilibrium states (10-state)
xEq_acc = [0;0;zEq_acc(1);zeros(7,1)];
xEq_dip = [0;0;zEq_dip(1);zeros(7,1)];
uEq = zeros(nu,1);

delta = 1e-5;

% Linearize all three
[A_acc, B_acc] = lin10(f_acc, xEq_acc, uEq, delta);
[A_dip_corr, B_dip_corr] = lin10(f_dip, xEq_dip, uEq, delta);
[A_dip_nocorr, B_dip_nocorr] = lin10(f_dip_nocorr, xEq_dip, uEq, delta);

%% Display comparison
fprintf('\n--- A matrix comparison (force-related rows 6-10) ---\n');
labels = {'x','y','z','r','p','vx','vy','vz','wx','wy'};
force_rows = 6:10;
force_labels = {'dvx','dvy','dvz','dwx','dwy'};

fprintf('\n%-6s', '');
for j = 1:nx, fprintf('%10s', labels{j}); end
fprintf('\n');

for i = 1:5
    row = force_rows(i);
    fprintf('\n%s (Accurate):\n', force_labels{i});
    fprintf('  Acc:   '); fprintf('%10.2f', A_acc(row,:)); fprintf('\n');
    fprintf('  Dip:   '); fprintf('%10.2f', A_dip_nocorr(row,:)); fprintf('\n');
    fprintf('  Corr:  '); fprintf('%10.2f', A_dip_corr(row,:)); fprintf('\n');
    fprintf('  Err%%:  ');
    for j = 1:nx
        if abs(A_acc(row,j)) > 1e-3
            err_pct = 100*(A_dip_corr(row,j) - A_acc(row,j))/A_acc(row,j);
            fprintf('%9.1f%%', err_pct);
        else
            fprintf('%10s', '-');
        end
    end
    fprintf('\n');
end

fprintf('\n--- B matrix comparison ---\n');
fprintf('  Acc norm:    %.4f\n', norm(B_acc));
fprintf('  Dip norm:    %.4f\n', norm(B_dip_nocorr));
fprintf('  Corr norm:   %.4f\n', norm(B_dip_corr));

% Eigenvalue comparison
eig_acc = eig(A_acc);
eig_dip = eig(A_dip_nocorr);
eig_corr = eig(A_dip_corr);

fprintf('\n--- Open-loop eigenvalues (real parts) ---\n');
fprintf('  Accurate:    '); fprintf('%.1f ', sort(real(eig_acc), 'descend')); fprintf('\n');
fprintf('  Dipole:      '); fprintf('%.1f ', sort(real(eig_dip), 'descend')); fprintf('\n');
fprintf('  Corrected:   '); fprintf('%.1f ', sort(real(eig_corr), 'descend')); fprintf('\n');

fprintf('\n--- A matrix error norms ---\n');
fprintf('  ||A_dip - A_acc||:  %.4f\n', norm(A_dip_nocorr - A_acc));
fprintf('  ||A_corr - A_acc||: %.4f\n', norm(A_dip_corr - A_acc));
fprintf('  Improvement:        %.1fx\n', norm(A_dip_nocorr - A_acc) / max(norm(A_dip_corr - A_acc), 1e-10));

%% Also test if Accurate self-stabilizes with umax=1 (LQR check)
fprintf('\n--- Controllability check (umax=1) ---\n');
C_mat = ctrb(A_acc, B_acc);
fprintf('  Controllability rank: %d / %d\n', rank(C_mat), nx);

% LQR for the Accurate model
Q = diag([1e4,1e4,1e6,1e3,1e3,1e2,1e2,1e3,1e2,1e2]);
R = eye(nu) * 1e0;
try
    [K_lqr, ~, eig_cl] = lqr(A_acc, B_acc, Q, R);
    fprintf('  LQR closed-loop eigenvalues (real): ');
    fprintf('%.1f ', sort(real(eig_cl), 'descend'));
    fprintf('\n');

    % Check required control at equilibrium with 1mm perturbation
    dx = [0;0;0.001;0;0;zeros(5,1)];
    u_lqr = -K_lqr * dx;
    fprintf('  LQR control for 1mm z-perturbation: |u|=%.3f (limit=1)\n', norm(u_lqr));
    fprintf('  Max |u_i|: %.3f\n', max(abs(u_lqr)));
catch e
    fprintf('  LQR failed: %s\n', e.message);
end


%% Helper functions
function dx = dynamics10(x10, u, params, modelId, m, I_vec, g)
    x12 = [x10(1:5); 0; x10(6:10); 0];  % insert yaw=0, wz=0
    [fx,fy,fz,tx,ty,~] = computeForceAndTorque(x12, u, params, modelId);
    dx = [x10(6);x10(7);x10(8);x10(9);x10(10);
          fx/m; fy/m; fz/m-g; tx/I_vec(1); ty/I_vec(2)];
end

function dx = dynamics10_corrected(x10, u, params_dip, m, I_vec, g)
    x12 = [x10(1:5); 0; x10(6:10); 0];
    [fx,fy,fz,tx,ty,~] = computeForceAndTorque(x12, u, params_dip, MaglevModel.Dipole);

    % Apply gradient correction
    if isfield(params_dip, 'dipole_corr')
        K = params_dip.dipole_corr.K;
        zEq = params_dip.dipole_corr.zEq;
        delta_q = [x10(1); x10(2); x10(3)-zEq; x10(4); x10(5)];
        corr = K * delta_q;
        fx = fx + corr(1); fy = fy + corr(2); fz = fz + corr(3);
        tx = tx + corr(4); ty = ty + corr(5);
    end

    dx = [x10(6);x10(7);x10(8);x10(9);x10(10);
          fx/m; fy/m; fz/m-g; tx/I_vec(1); ty/I_vec(2)];
end

function [A, B] = lin10(f, xEq, uEq, delta)
    nx = length(xEq); nu = length(uEq);
    A = zeros(nx);
    B = zeros(nx, nu);
    for j = 1:nx
        dx = zeros(nx,1); dx(j) = delta;
        A(:,j) = (f(xEq+dx,uEq) - f(xEq-dx,uEq)) / (2*delta);
    end
    for j = 1:nu
        du = zeros(nu,1); du(j) = delta;
        B(:,j) = (f(xEq,uEq+du) - f(xEq,uEq-du)) / (2*delta);
    end
end
