function model = get_maggy_model(modelId, opts)
% GET_MAGGY_MODEL  Build AcadosModel for the full-order maglev system.
%
%   model = get_maggy_model(modelId)
%   model = get_maggy_model(modelId, use_luts=true)
%   model = get_maggy_model(modelId, use_luts=false)
%
%   modelId selects the physics model:
%     MaglevModel.Fast     - Wire-loop field model
%     MaglevModel.Accurate - Current-sheet field model
%
%   use_luts (name-value, optional):
%     true  - Pre-built LUTs (requires MX symbols)
%     false - Analytical elliptic integrals (uses SX symbols, faster codegen)
%     If omitted, uses params.lut_opts.enabled from parameter file.
%
%   Parameters are loaded internally (with solenoid correction for Fast).

arguments
    modelId (1,1) MaglevModel = MaglevModel.Accurate
    opts.use_luts logical
end

import casadi.*

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

nx = 10;
nu = 4;

% Always use MX for outer symbols. This preserves CasADi Function/map
% structure in the expression graph — acados generates loops instead of
% unrolled code. Inner Functions use SX for efficient scalar AD.
x    = MX.sym('x',    nx);
u    = MX.sym('u',    nu);
xdot = MX.sym('xdot', nx);

if ~use_luts && modelId == MaglevModel.Accurate
    warning("Acados model probably will not compile, as dynamics are to complex, consider enabling luts or using the fast model");
end

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

end
