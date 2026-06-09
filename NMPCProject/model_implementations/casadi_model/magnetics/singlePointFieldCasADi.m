function f_point = singlePointFieldCasADi(params)
% SINGLEPOINTFIELDCASADI  CasADi Function for the total magnetic field at a
% single query point in world coordinates.
%
%   f_point = singlePointFieldCasADi(params)
%
% Returns a casadi.Function with signature
%   f_point(p, u) -> b
% where
%   p (3x1)   query point in world coordinates [px; py; pz]
%   u (nu x1) solenoid currents
%   b (3x1)   magnetic field [bx; by; bz]
%
% Identical superposition logic to computeMagneticFieldCasADi.m, but exposed
% as a scalar (single-point) Function so callers can invoke
% f_point.map(magnet_n) and have CasADi emit the per-point body once in the
% generated C, rather than inlining it magnet_n times.

    import casadi.*

    nu = length(params.solenoids.r);

    p = SX.sym('p', 3);
    u = SX.sym('u', nu);

    px = p(1);
    py = p(2);
    pz = p(3);

    mu0 = params.physical.mu0;

    bx = SX.zeros(1, 1);
    by = SX.zeros(1, 1);
    bz = SX.zeros(1, 1);

    % Permanent magnets
    n_perm = length(params.permanent.r);
    for i = 1:n_perm
        I_perm = params.permanent.J / mu0 * params.permanent.l(i);

        [dbx, dby, dbz] = computeCircularWireFieldCasADi( ...
            px - params.permanent.x(i), ...
            py - params.permanent.y(i), ...
            pz - params.permanent.z(i), ...
            params.permanent.r(i), I_perm, mu0);

        bx = bx + dbx;
        by = by + dby;
        bz = bz + dbz;
    end

    % Solenoids (current-controlled)
    n_sol = length(params.solenoids.r);
    for i = 1:n_sol
        [dbx, dby, dbz] = computeCircularWireFieldCasADi( ...
            px - params.solenoids.x(i), ...
            py - params.solenoids.y(i), ...
            pz - params.solenoids.z(i), ...
            params.solenoids.r(i), u(i), mu0);

        bx = bx + dbx * params.solenoids.nw;
        by = by + dby * params.solenoids.nw;
        bz = bz + dbz * params.solenoids.nw;
    end

    f_point = Function('f_point', {p, u}, {[bx; by; bz]}, ...
        {'p', 'u'}, {'b'});
end
