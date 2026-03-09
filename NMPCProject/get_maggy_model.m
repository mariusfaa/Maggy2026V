function model = get_maggy_model(params, modelId, opts)
% GET_MAGGY_MODEL  Build AcadosModel for the maglev system.
%   model = get_maggy_model(params, modelId)
%   model = get_maggy_model(params, modelId, opts)
%
%   opts.field_method - 'lut' (default) or 'analytical'

import casadi.*

if nargin < 2, modelId = MaglevModel.Accurate; end
if nargin < 3, opts = struct(); end

% Set field computation method
if isfield(opts, 'field_method')
    params.field_method = opts.field_method;
end

% Build LUTs only if using LUT-based field computation
if ~isfield(params, 'field_method') || strcmp(params.field_method, 'lut')
    fprintf('--- Building LUTs ---\n');
    params.luts = buildLuts(params);
else
    fprintf('--- Using analytical field computation (no LUTs) ---\n');
end

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
