function model = get_maggy_model(params)

import casadi.*

nx   = 12;
nu   = 4;
x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

fprintf('--- Setting up casadi dynamics ---\n');
f_expl = maglevSystemDynamicsCasADi(x, u, params);

model = AcadosModel();
model.name        = 'maglev_sim';
model.x           = x;
model.u           = u;
model.xdot        = xdot;
model.f_impl_expr = xdot - f_expl;
model.f_expl_expr = f_expl;

end