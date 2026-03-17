simSetup;
import casadi.*

modelId = MaglevModel.Fast;
opts.use_luts = false;

% Load params configured for this model
params = load_params(modelId);

% Override LUT setting if provided
if isfield(opts, 'use_luts')
    params.lut_opts.enabled = opts.use_luts;
end

use_luts = params.lut_opts.enabled;

% Build LUTs if enabled
if use_luts
    fprintf('--- Building LUTs (model=%s) ---\n', string(modelId));
    params.luts = buildLuts(params, modelId);
else
    fprintf('--- Using analytical field computation (no LUTs) ---\n');
end

nx = 12;
nu = 4;
nx_red = 10;  % remove indices 6 and 12

x = SX.sym('x', nx_red);   % actual free states
u     = SX.sym('u', nu);
xdot  = SX.sym('xdot', nx_red);

fprintf('--- Setting up CasADi dynamics (model=%s, lut=%d) ---\n', ...
    string(modelId), use_luts);
f_expl = maglevSystemDynamicsReduced_casadi(x, u, params, modelId);

model = AcadosModel();
model.name        = 'maglev_sim';
model.x           = x;
model.u           = u;
model.xdot        = xdot;
model.f_impl_expr = xdot - f_expl;
model.f_expl_expr = f_expl;

f = f_expl;
J_x = jacobian(f, x);
J_u = jacobian(f, u);
J_xdot = jacobian(f, xdot);
H_xx = jacobian(J_x(:), x);   % second order, large
H_xu = jacobian(J_x(:), u);

fprintf('f=f_expl\n');
fprintf('f           nodes: %d\n', f.n_nodes());
fprintf('J_x         nodes: %d\n', J_x.n_nodes());
fprintf('J_u         nodes: %d\n', J_u.n_nodes());
fprintf('J_xdot      nodes: %d\n', J_xdot.n_nodes());
fprintf('H_xx        nodes: %d\n', H_xx.n_nodes());
fprintf('H_xu        nodes: %d\n', H_xu.n_nodes());