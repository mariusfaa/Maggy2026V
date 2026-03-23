function [K, B_corr] = computeDipoleGradientCorrection(params_dip, zEq_dip)
% COMPUTEDIPOLEGRADIENTCORRECTION  Compute force/torque Jacobian correction.
%
%   [K, B_corr] = computeDipoleGradientCorrection(params_dip, zEq_dip)
%
% Returns:
%   K      - 5x5 state Jacobian correction matrix
%   B_corr - 5x4 input Jacobian correction matrix
%
% Applied as:
%   [fx;fy;fz;tx;ty]_corrected = [fx;fy;fz;tx;ty]_dipole
%                                + K * [x; y; z-zEq; roll; pitch]
%                                + B_corr * u
%
% K = dF/dx_accurate - dF/dx_dipole
% B_corr = dF/du_accurate - dF/du_dipole

    % Get Accurate reference
    params_acc = load_params(MaglevModel.Accurate);
    [zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);
    zEq_a = zEq_acc(1);

    % Equilibrium states (12-state for MATLAB computeForceAndTorque)
    x_eq_acc = [0; 0; zEq_a; zeros(9,1)];
    x_eq_dip = [0; 0; zEq_dip; zeros(9,1)];
    u0 = zeros(4,1);

    delta = 1e-5;

    % === State Jacobian (5x5) ===
    perturb_idx = [1, 2, 3, 4, 5];  % x, y, z, roll, pitch
    J_acc = zeros(5, 5);
    J_dip = zeros(5, 5);

    for j = 1:5
        idx = perturb_idx(j);

        x_plus = x_eq_acc; x_plus(idx) = x_plus(idx) + delta;
        x_minus = x_eq_acc; x_minus(idx) = x_minus(idx) - delta;
        [fxp,fyp,fzp,txp,typ,~] = computeForceAndTorque(x_plus, u0, params_acc, MaglevModel.Accurate);
        [fxm,fym,fzm,txm,tym,~] = computeForceAndTorque(x_minus, u0, params_acc, MaglevModel.Accurate);
        J_acc(:,j) = ([fxp;fyp;fzp;txp;typ] - [fxm;fym;fzm;txm;tym]) / (2*delta);

        x_plus_d = x_eq_dip; x_plus_d(idx) = x_plus_d(idx) + delta;
        x_minus_d = x_eq_dip; x_minus_d(idx) = x_minus_d(idx) - delta;
        [fxp,fyp,fzp,txp,typ,~] = computeForceAndTorque(x_plus_d, u0, params_dip, MaglevModel.Dipole);
        [fxm,fym,fzm,txm,tym,~] = computeForceAndTorque(x_minus_d, u0, params_dip, MaglevModel.Dipole);
        J_dip(:,j) = ([fxp;fyp;fzp;txp;typ] - [fxm;fym;fzm;txm;tym]) / (2*delta);
    end

    K = J_acc - J_dip;

    % === Input Jacobian (5x4) ===
    du = 1e-3;  % current perturbation [A]
    B_acc = zeros(5, 4);
    B_dip = zeros(5, 4);

    for j = 1:4
        u_plus = u0; u_plus(j) = u_plus(j) + du;
        u_minus = u0; u_minus(j) = u_minus(j) - du;

        [fxp,fyp,fzp,txp,typ,~] = computeForceAndTorque(x_eq_acc, u_plus, params_acc, MaglevModel.Accurate);
        [fxm,fym,fzm,txm,tym,~] = computeForceAndTorque(x_eq_acc, u_minus, params_acc, MaglevModel.Accurate);
        B_acc(:,j) = ([fxp;fyp;fzp;txp;typ] - [fxm;fym;fzm;txm;tym]) / (2*du);

        [fxp,fyp,fzp,txp,typ,~] = computeForceAndTorque(x_eq_dip, u_plus, params_dip, MaglevModel.Dipole);
        [fxm,fym,fzm,txm,tym,~] = computeForceAndTorque(x_eq_dip, u_minus, params_dip, MaglevModel.Dipole);
        B_dip(:,j) = ([fxp;fyp;fzp;txp;typ] - [fxm;fym;fzm;txm;tym]) / (2*du);
    end

    B_corr = B_acc - B_dip;

    % Print diagnostics
    labels = {'fx','fy','fz','tx','ty'};
    state_labels = {'x','y','z','roll','pitch'};

    fprintf('  State Jacobian correction K (J_acc - J_dip):\n');
    for i = 1:5
        for j = 1:5
            if abs(K(i,j)) > 1e-6
                fprintf('    d%s/d%s: acc=%.4f, dip=%.4f, corr=%.4f\n', ...
                    labels{i}, state_labels{j}, J_acc(i,j), J_dip(i,j), K(i,j));
            end
        end
    end

    fprintf('  Input Jacobian correction B_corr (dF/du_acc - dF/du_dip):\n');
    for i = 1:5
        for j = 1:4
            if abs(B_corr(i,j)) > 1e-6
                fprintf('    d%s/du%d: acc=%.6f, dip=%.6f, corr=%.6f\n', ...
                    labels{i}, j, B_acc(i,j), B_dip(i,j), B_corr(i,j));
            end
        end
    end
end
