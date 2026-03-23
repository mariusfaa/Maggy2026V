addpath(genpath('model_matlab'));
addpath(genpath('system_parameters'));
addpath(genpath('utilities'));

fprintf('=== Recalibration Test ===\n');
params_dip = load_params(MaglevModel.Dipole);
params_acc = load_params(MaglevModel.Accurate);

[zEq_dip, ~, ~, ~] = computeSystemEquilibria(params_dip, MaglevModel.Dipole);
[zEq_acc, ~, ~, ~] = computeSystemEquilibria(params_acc, MaglevModel.Accurate);
fprintf('\nDipole eq:   z = %.4f mm\n', zEq_dip(1)*1e3);
fprintf('Accurate eq: z = %.4f mm\n', zEq_acc(1)*1e3);

% Compare forces with solenoid input
x_eq = [0;0;zEq_acc(1);zeros(9,1)];
u_test = 0.5*ones(4,1);
[~,~,fz_dip] = computeForceAndTorque(x_eq, u_test, params_dip, MaglevModel.Dipole);
[~,~,fz_acc] = computeForceAndTorque(x_eq, u_test, params_acc, MaglevModel.Accurate);
fprintf('\nAt eq, u=0.5A: fz_acc=%.6f, fz_dip=%.6f, err=%.4f mN\n', ...
    fz_acc, fz_dip, (fz_dip-fz_acc)*1e3);

% Also check lateral force sensitivity
x_lat = [0.002;0;zEq_acc(1);zeros(9,1)];  % 2mm x offset
[fx_dip,~,~] = computeForceAndTorque(x_lat, zeros(4,1), params_dip, MaglevModel.Dipole);
[fx_acc,~,~] = computeForceAndTorque(x_lat, zeros(4,1), params_acc, MaglevModel.Accurate);
fprintf('At x=2mm: fx_acc=%.6f, fx_dip=%.6f, ratio=%.3f\n', fx_acc, fx_dip, fx_dip/fx_acc);
