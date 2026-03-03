function model = get_maggy_model(params)

import casadi.*

% Build luts
fprintf('--- Building LUTS ---\n');
opts = struct();
opts.rho_max = 0.1;      % 10cm
opts.z_max = 0.06;       % 6cm
opts.method = 'linear';  % 'linear' is much faster in generated C code than 'bspline'
opts.N_rho = 100;        % denser grid compensates for linear interpolation
opts.N_z = 100;
params.luts = buildCurrentSheetLuts(params, opts);


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