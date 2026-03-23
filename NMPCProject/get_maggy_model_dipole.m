function model = get_maggy_model_dipole()
% GET_MAGGY_MODEL_DIPOLE  Build AcadosModel using the dipole approximation.
%
%   model = get_maggy_model_dipole()
%
%   Uses magnetic dipole approximation for all sources (4 permanent magnets
%   + 4 solenoids) and the levitating magnet. No LUTs or elliptic integrals
%   needed — this is the fastest model variant.
%
%   The interface matches get_maggy_model() for drop-in use with acados.

import casadi.*

% Load params (no model-specific correction needed for dipole)
params = load_params(MaglevModel.Dipole);

nx = 10;
nu = 4;

x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

fprintf('--- Setting up CasADi dynamics (model=Dipole) ---\n');
f_expl = maglevSystemDynamicsDipole_casadi(x, u, params);

model = AcadosModel();
model.name        = 'maglev_sim';
model.x           = x;
model.u           = u;
model.xdot        = xdot;
model.f_impl_expr = xdot - f_expl;
model.f_expl_expr = f_expl;

end
