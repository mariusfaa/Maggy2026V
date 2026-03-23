function [c_perm, c_sol] = computeDipoleCorrectionFactor(params)
% COMPUTEDIPOLECORRECTIONFACTOR  Calibrate dipole model to match Accurate.
%
%   [c_perm, c_sol] = computeDipoleCorrectionFactor(params)
%
% Finds two correction factors:
%   c_perm - scales permanent magnet J (matches eq force + lateral stiffness)
%   c_sol  - scales solenoid nw (matches solenoid force sensitivity)
%
% Both are optimized to match the Accurate model's forces at and near
% the equilibrium height, including off-axis forces.

    % Get reference from Accurate model
    params_acc = load_params(MaglevModel.Accurate);
    [zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);
    zEq = zEq_acc(1);

    fprintf('  Target equilibrium (Accurate): z = %.4f mm\n', zEq*1e3);

    % Reference forces from Accurate model at multiple operating points
    u0 = zeros(4,1);
    u_test = ones(4,1);

    % Point 1: On-axis equilibrium, u=0
    x_eq = [0; 0; zEq; zeros(9,1)];
    [~, ~, fz_acc_eq] = computeForceAndTorque(x_eq, u0, params_acc, MaglevModel.Accurate);

    % Point 2: On-axis equilibrium, u=1A (solenoid sensitivity)
    [~, ~, fz_acc_u1] = computeForceAndTorque(x_eq, u_test, params_acc, MaglevModel.Accurate);
    dfz_acc = fz_acc_u1 - fz_acc_eq;

    % Point 3: Off-axis x=2mm (lateral force from permanent magnets)
    x_lat = [0.002; 0; zEq; zeros(9,1)];
    [fx_acc_lat, ~, ~] = computeForceAndTorque(x_lat, u0, params_acc, MaglevModel.Accurate);

    % Point 4: z offset +2mm (vertical stiffness)
    x_zoff = [0; 0; zEq + 0.002; zeros(9,1)];
    [~, ~, fz_acc_zoff] = computeForceAndTorque(x_zoff, u0, params_acc, MaglevModel.Accurate);

    fprintf('  Accurate: fz_eq=%.6f, dfz_sol=%.6f, fx_lat=%.6f, fz_z+2mm=%.6f\n', ...
        fz_acc_eq, dfz_acc, fx_acc_lat, fz_acc_zoff);

    targets = struct('fz_eq', fz_acc_eq, 'dfz_sol', dfz_acc, ...
                     'fx_lat', fx_acc_lat, 'fz_zoff', fz_acc_zoff, ...
                     'zEq', zEq);

    % Optimize [c_perm, c_sol] jointly
    options = optimoptions('fmincon', 'Display', 'off', ...
        'OptimalityTolerance', 1e-12, 'StepTolerance', 1e-12);
    c0 = [0.5; 1.0];
    lb = [0.1; 0.1];
    ub = [5; 5];

    c_opt = fmincon(@(c) objfun(c, params, targets), ...
        c0, [], [], [], [], lb, ub, [], options);

    c_perm = c_opt(1);
    c_sol  = c_opt(2);

    % Verify
    params_corr = applyCorrection(params, c_perm, c_sol);
    [fz_eq, dfz, fx_lat, fz_zoff] = evalForces(params_corr, targets.zEq);

    fprintf('  c_perm = %.6f, c_sol = %.6f\n', c_perm, c_sol);
    fprintf('  fz_eq:    acc=%.6f, dip=%.6f, err=%.2e\n', fz_acc_eq, fz_eq, abs(fz_acc_eq - fz_eq));
    fprintf('  dfz_sol:  acc=%.6f, dip=%.6f, err=%.2e\n', dfz_acc, dfz, abs(dfz_acc - dfz));
    fprintf('  fx_lat:   acc=%.6f, dip=%.6f, ratio=%.3f\n', fx_acc_lat, fx_lat, fx_lat/fx_acc_lat);
    fprintf('  fz_z+2mm: acc=%.6f, dip=%.6f, ratio=%.3f\n', fz_acc_zoff, fz_zoff, fz_zoff/fz_acc_zoff);
end

function J = objfun(c, params, tgt)
    params_c = applyCorrection(params, c(1), c(2));
    [fz_eq, dfz, fx_lat, fz_zoff] = evalForces(params_c, tgt.zEq);

    % Weighted objectives:
    %   - equilibrium match (highest priority)
    %   - solenoid sensitivity
    %   - lateral force (important for stability)
    %   - vertical stiffness
    J = 100 * (fz_eq - tgt.fz_eq)^2 ...      % eq match
      +       (dfz - tgt.dfz_sol)^2 ...        % solenoid sensitivity
      +  10 * (fx_lat - tgt.fx_lat)^2 ...      % lateral stiffness
      +       (fz_zoff - tgt.fz_zoff)^2;       % vertical stiffness
end

function [fz_eq, dfz_sol, fx_lat, fz_zoff] = evalForces(params, zEq)
    u0 = zeros(4,1);
    u1 = ones(4,1);

    x_eq = [0; 0; zEq; zeros(9,1)];
    [~, ~, fz_eq] = computeForceAndTorque(x_eq, u0, params, MaglevModel.Dipole);

    [~, ~, fz_u1] = computeForceAndTorque(x_eq, u1, params, MaglevModel.Dipole);
    dfz_sol = fz_u1 - fz_eq;

    x_lat = [0.002; 0; zEq; zeros(9,1)];
    [fx_lat, ~, ~] = computeForceAndTorque(x_lat, u0, params, MaglevModel.Dipole);

    x_zoff = [0; 0; zEq + 0.002; zeros(9,1)];
    [~, ~, fz_zoff] = computeForceAndTorque(x_zoff, u0, params, MaglevModel.Dipole);
end

function p = applyCorrection(params, c_perm, c_sol)
    p = params;
    p.permanent.J = c_perm * params.permanent.J;
    p.solenoids.nw = c_sol * params.solenoids.nw;
end
