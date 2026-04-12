function model = getSimModel(n, n_axial,modelId)

if nargin < 1 || isempty(n),       n       = 16; end
if nargin < 2 || isempty(n_axial), n_axial = 1;  end
if nargin < 3 || isempty(modelId), modelId = MaglevModel.Accurate;  end

import casadi.*

%fprintf('--- Setting up sim model ---\n');
% Load params (Accurate model needs no corrections)
parameters_maggy_V4;
params.magnet.n       = n;
params.magnet.n_axial = n_axial;

if modelId == MaglevModel.Fast
    params.solenoids.r = params.solenoids.r * computeSolenoidRadiusCorrectionFactor(params,modelId);
end

nx = 10;
nu = 4;

x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

f_expl = maglevSystemDynamicsReduced_casadi(x, u, params, modelId);

model = AcadosModel();
model.name        = sprintf('maglev_sim_model_%d_%d',n, n_axial);
model.x           = x;
model.u           = u;
model.xdot        = xdot;
model.f_impl_expr = xdot - f_expl;
model.f_expl_expr = f_expl;

end