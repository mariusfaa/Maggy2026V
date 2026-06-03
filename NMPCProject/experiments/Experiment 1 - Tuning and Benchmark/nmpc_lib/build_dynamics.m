function [f_func, params] = build_dynamics(magnet_n)
% BUILD_DYNAMICS  Build the 12-state CasADi dynamics function for a given
% levitating-magnet discretization n.
%
%   [f_func, params] = build_dynamics(magnet_n)
%
% Inputs
%   magnet_n   integer >= 3, replaces params.magnet.n
%
% Outputs
%   f_func     casadi.Function with signature f_func(x12, u4) -> dx12
%              evaluated using the 'fast' magnetic-field model with the
%              solenoid radius correction factor applied.
%   params     the params struct used to build f_func (post-correction).
%
% Notes
%   The correction factor is recomputed for each unique magnet_n so that
%   the plant and controller solenoid geometries are consistent with their
%   own discretizations. Results are cached in a persistent map keyed on
%   magnet_n to avoid the (slow) fmincon call on repeated invocations.

    import casadi.*

    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType','int32','ValueType','any');
    end

    key = int32(magnet_n);
    if isKey(cache, key)
        entry  = cache(key);
        f_func = entry.f_func;
        params = entry.params;
        return;
    end

    % --- Load defaults via the existing script (sets `params` in this scope) ---
    parameters_maggy_V4;                                         %#ok<NODEF>

    % --- Override magnet.n and apply the 'fast' solenoid correction ---
    params.magnet.n = double(magnet_n);
    cf = computeSolenoidRadiusCorrectionFactor(params, 'fast');
    params.solenoids.r = cf * params.solenoids.r;

    % --- Build CasADi dynamics function ---
    nx_full = 12; nu = 4;
    x_sym = SX.sym('x_full', nx_full);
    u_sym = SX.sym('u', nu);
    f_expr = maglevSystemDynamicsCasADi(x_sym, u_sym, params);
    f_func = casadi.Function(sprintf('f_full_n%d', magnet_n), ...
                             {x_sym, u_sym}, {f_expr});

    cache(key) = struct('f_func', f_func, 'params', params);
end
