function model = getSimModel()

import casadi.*

% Setup model object
modelId = MaglevModel.Accurate;

fprintf('--- Setting up sim model ---\n');
% Load params configured for this model
params = load_params(modelId);
params.magnet.n     = 16; % determined by trail and error, "fast buildtime vs high accuracy"
params.magnet.n_axial = 1;

nx = 10;
nu = 4;

x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

f_expl = maglevSystemDynamicsReduced_casadi(x, u, params, modelId);

model = AcadosModel();
model.name        = 'maglev_sim_model';
model.x           = x;
model.u           = u;
model.xdot        = xdot;
model.f_impl_expr = xdot - f_expl;
model.f_expl_expr = f_expl;

end