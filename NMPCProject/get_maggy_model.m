function model = get_maggy_model(params, modelId)

import casadi.*

if nargin < 2, modelId = MaglevModel.Accurate; end

% Build LUTs using modular implementation
fprintf('--- Building LUTs ---\n');
params.luts = buildLuts(params);

nx   = 12;
nu   = 4;
x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

fprintf('--- Setting up CasADi dynamics (model=%s) ---\n', string(modelId));
f_expl = maglevSystemDynamics_casadi(x, u, params, modelId);

model = AcadosModel();
model.name        = 'maglev_sim';
model.x           = x;
model.u           = u;
model.xdot        = xdot;
model.f_impl_expr = xdot - f_expl;
model.f_expl_expr = f_expl;

end
