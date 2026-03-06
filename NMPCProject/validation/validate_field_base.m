%% validate_field_base.m
% Step 4: Validate CasADi computeFieldBase_casadi vs MATLAB computeFieldBase
%
% Tests the .map()-based field computation against the MATLAB reference.
%
% Pass criteria: max relative error < 1%

% clear; clc;
addpath(genpath('..'));

import casadi.*

%% Setup
load_params(MaglevModel.Accurate);

luts = buildLuts(params);
params.luts = luts;

%% Test parameters
nTest = 500;

%% Build CasADi function for evaluation
px_sym = MX.sym('px', 1, nTest);
py_sym = MX.sym('py', 1, nTest);
pz_sym = MX.sym('pz', 1, nTest);
u_sym  = MX.sym('u', 4, 1);

[bx_sym, by_sym, bz_sym] = computeFieldBase_casadi(px_sym, py_sym, pz_sym, u_sym, params);

field_func = Function('field', {px_sym, py_sym, pz_sym, u_sym}, ...
                               {bx_sym, by_sym, bz_sym});

fprintf('Symbolic output sizes: bx=%dx%d, by=%dx%d, bz=%dx%d\n', ...
    size(bx_sym,1), size(bx_sym,2), size(by_sym,1), size(by_sym,2), ...
    size(bz_sym,1), size(bz_sym,2));

%% Test points
rng(42);

u_configs = {
    [0; 0; 0; 0],
    [1; 0; 0; 0],
    [1; -1; 1; -1],
    randn(4,1).*2 - 1,
};

xy_max = params.lut_opts.perm3d_xy * 0.9;
z_lo   = params.lut_opts.perm3d_z(1) + 0.001;
z_hi   = params.lut_opts.perm3d_z(2) - 0.001;

x_test = (2*rand(1, nTest) - 1) * xy_max;
y_test = (2*rand(1, nTest) - 1) * xy_max;
z_test = z_lo + rand(1, nTest) * (z_hi - z_lo);

%% Head-to-head diagnostic
fprintf('\n--- HEAD-TO-HEAD (u=0, first 3 pts) ---\n');
u_zero = [0;0;0;0];

[bx_dm, by_dm, bz_dm] = field_func(x_test, y_test, z_test, u_zero);
bx_ff = full(bx_dm); by_ff = full(by_dm); bz_ff = full(bz_dm);

fprintf('field_func output sizes: bx=%dx%d, by=%dx%d, bz=%dx%d\n', ...
    size(bx_ff,1), size(bx_ff,2), size(by_ff,1), size(by_ff,2), ...
    size(bz_ff,1), size(bz_ff,2));

[bx_ml, by_ml, bz_ml] = computeFieldBase( ...
    x_test, y_test, z_test, u_zero, params, MaglevModel.Accurate);

for k = 1:3
    r_d = full(luts.perm_3d([x_test(k); y_test(k); z_test(k)]));
    fprintf('pt %d (x=%.4f, y=%.4f, z=%.4f):\n', k, x_test(k), y_test(k), z_test(k));
    fprintf('  perm_3d:    bx=%.4e  by=%.4e  bz=%.4e\n', r_d(1), r_d(2), r_d(3));
    fprintf('  field_func: bx=%.4e  by=%.4e  bz=%.4e\n', bx_ff(k), by_ff(k), bz_ff(k));
    fprintf('  MATLAB ref: bx=%.4e  by=%.4e  bz=%.4e\n', bx_ml(k), by_ml(k), bz_ml(k));
end
fprintf('--- END HEAD-TO-HEAD ---\n');

%% Main test
fprintf('\n=== Step 4: CasADi Field Base (.map) vs MATLAB ===\n');
all_pass = true;

for c = 1:length(u_configs)
    u = u_configs{c};

    [bx_dm, by_dm, bz_dm] = field_func(x_test, y_test, z_test, u);
    bx_casadi = full(bx_dm);
    by_casadi = full(by_dm);
    bz_casadi = full(bz_dm);

    [bx_ref, by_ref, bz_ref] = computeFieldBase( ...
        x_test, y_test, z_test, u, params, MaglevModel.Accurate);

    fields = {'bx', 'by', 'bz'};
    casadi_vals = {bx_casadi, by_casadi, bz_casadi};
    ref_vals = {bx_ref, by_ref, bz_ref};

    fprintf('\nConfig %d: u = [%.2f, %.2f, %.2f, %.2f]\n', ...
        c, u(1), u(2), u(3), u(4));

    for f = 1:3
        eps_f = max(abs(ref_vals{f})) * 0.01;
        mixed_err = abs(casadi_vals{f} - ref_vals{f}) ./ (abs(ref_vals{f}) + eps_f);
        p99 = prctile(mixed_err*100, 99);
        fprintf('  %s - max: %.4f%%  p99: %.4f%%  mean: %.4f%%\n', ...
            fields{f}, max(mixed_err)*100, p99, mean(mixed_err)*100);
        if max(mixed_err) >= 0.01
            [~, widx] = max(mixed_err);
            fprintf('    worst pt %d: (%.4f,%.4f,%.4f) casadi=%.4e ref=%.4e\n', ...
                widx, x_test(widx), y_test(widx), z_test(widx), ...
                casadi_vals{f}(widx), ref_vals{f}(widx));
            all_pass = false;
        end
    end
end

if all_pass
    fprintf('\nPASS\n');
else
    fprintf('\nFAIL\n');
end
