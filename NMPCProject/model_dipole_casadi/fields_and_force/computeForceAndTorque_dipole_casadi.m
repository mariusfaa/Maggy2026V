function [fx,fy,fz,tx,ty,tz] = computeForceAndTorque_dipole_casadi(x, u, params)
% COMPUTEFORCEANDTORQUE_DIPOLE_CASADI  Force & torque using dipole source fields.
%
% Uses the same surface-integration approach as the Fast model (circumferential
% points on the levitating magnet), but replaces the elliptic-integral field
% computation with the closed-form magnetic dipole formula.
%
% This is faster than the Fast model because the dipole field formula is
% ~5 operations per source vs ~100+ for elliptic integrals, while maintaining
% the same accuracy for force/torque integration.
%
% State (10x1): [x; y; z; roll; pitch; vx; vy; vz; wx; wy]

    import casadi.*

    assert(numel(x) == 10);

    nRadial  = params.magnet.n;
    magnet_r = params.magnet.r;
    magnet_l = params.magnet.l;
    K = -params.magnet.J / params.physical.mu0;

    theta = linspace(0, 2*pi - 2*pi/nRadial, nRadial);
    dtheta = 2*pi / nRadial;
    nEval = nRadial;

    cos_th_vals = cos(theta);
    sin_th_vals = sin(theta);

    % =====================================================================
    % Function A: f_pos — rotate surface point + add position offset
    % (identical to Fast model)
    % =====================================================================
    alpha_s  = SX.sym('alpha');
    beta_s   = SX.sym('beta');
    posx_s   = SX.sym('posx');
    posy_s   = SX.sym('posy');
    posz_s   = SX.sym('posz');
    cos_th_s = SX.sym('cos_th');
    sin_th_s = SX.sym('sin_th');

    ca = cos(alpha_s); sa = sin(alpha_s);
    cb = cos(beta_s);  sb = sin(beta_s);
    % R = Ry(beta) * Rx(alpha)
    R11 = cb;     R12 = sa*sb;  R13 = ca*sb;
    R21 = 0;      R22 = ca;     R23 = -sa;
    R31 = -sb;    R32 = sa*cb;  R33 = ca*cb;

    lx = magnet_r * cos_th_s;
    ly = magnet_r * sin_th_s;

    px_s = R11*lx + R12*ly + posx_s;
    py_s = R21*lx + R22*ly + posy_s;
    pz_s = R31*lx + R32*ly + posz_s;

    f_pos = Function('f_pos', ...
        {alpha_s, beta_s, posx_s, posy_s, posz_s, cos_th_s, sin_th_s}, ...
        {px_s, py_s, pz_s});
    f_pos_map = f_pos.map(nEval);

    alpha_rep = repmat(x(4), 1, nEval);
    beta_rep  = repmat(x(5), 1, nEval);
    posx_rep  = repmat(x(1), 1, nEval);
    posy_rep  = repmat(x(2), 1, nEval);
    posz_rep  = repmat(x(3), 1, nEval);

    [px, py, pz] = f_pos_map(alpha_rep, beta_rep, posx_rep, posy_rep, posz_rep, ...
                              cos_th_vals, sin_th_vals);

    % =====================================================================
    % Compute field at the rotated surface points using DIPOLE model
    % =====================================================================
    [bx, by, bz] = computeFieldBase_dipole(px, py, pz, u, params);

    % =====================================================================
    % Function B: f_ft — force & torque cross products per point
    % (identical to Fast model)
    % =====================================================================
    alpha_s2  = SX.sym('alpha');
    beta_s2   = SX.sym('beta');
    bx_s      = SX.sym('bx');
    by_s      = SX.sym('by');
    bz_s      = SX.sym('bz');
    cos_th_s2 = SX.sym('cos_th');
    sin_th_s2 = SX.sym('sin_th');

    ca2 = cos(alpha_s2); sa2 = sin(alpha_s2);
    cb2 = cos(beta_s2);  sb2 = sin(beta_s2);
    R11b = cb2;    R12b = sa2*sb2;  R13b = ca2*sb2;
    R21b = 0;      R22b = ca2;      R23b = -sa2;
    R31b = -sb2;   R32b = sa2*cb2;  R33b = ca2*cb2;

    w = K * magnet_l * magnet_r * dtheta;

    tlx = -sin_th_s2;
    tly =  cos_th_s2;
    tgx = R11b*tlx + R12b*tly;
    tgy = R21b*tlx + R22b*tly;
    tgz = R31b*tlx + R32b*tly;

    Fx_s = w * (tgy * bz_s - tgz * by_s);
    Fy_s = w * (tgz * bx_s - tgx * bz_s);
    Fz_s = w * (tgx * by_s - tgy * bx_s);

    ngx = R13b;
    ngy = R23b;
    ngz = R33b;

    Tx_s = w * (ngy * bz_s - ngz * by_s);
    Ty_s = w * (ngz * bx_s - ngx * bz_s);
    Tz_s = w * (ngx * by_s - ngy * bx_s);

    f_ft = Function('f_ft', ...
        {alpha_s2, beta_s2, bx_s, by_s, bz_s, cos_th_s2, sin_th_s2}, ...
        {Fx_s, Fy_s, Fz_s, Tx_s, Ty_s, Tz_s});
    f_ft_map = f_ft.map(nEval);

    [Fx_all, Fy_all, Fz_all, Tx_all, Ty_all, Tz_all] = ...
        f_ft_map(alpha_rep, beta_rep, bx, by, bz, cos_th_vals, sin_th_vals);

    fx = sum2(Fx_all);
    fy = sum2(Fy_all);
    fz = sum2(Fz_all);
    tx = sum2(Tx_all);
    ty = sum2(Ty_all);
    tz = sum2(Tz_all);
end
