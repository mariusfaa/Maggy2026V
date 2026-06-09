function dx = maglevSystemDynamicsCasADiMapped(x, u, params)
% MAGLEVSYSTEMDYNAMICSCASADIMAPPED  Same dynamics as
% maglevSystemDynamicsCasADi, but the per-point magnetic-field integrand is
% built once as a casadi.Function and applied to the magnet_n circumference
% points via .map(n). This collapses the n-fold inlined arithmetic in the
% generated C to one body called n times.
%
% Inputs / outputs match maglevSystemDynamicsCasADi exactly so the two are
% drop-in replacements.

    import casadi.*

    %% Extract parameters
    mass   = params.magnet.m;
    I_vec  = params.magnet.I;
    g      = params.physical.g;

    magnet_r = params.magnet.r;
    magnet_l = params.magnet.l;
    magnet_J = params.magnet.J;
    magnet_n = params.magnet.n;
    mu0      = params.physical.mu0;

    %% Rotation matrix from Euler angles
    R = computeRotationMatrixCasADi(x(4), x(5), x(6));

    %% Discretize levitating magnet circumference
    theta = linspace(0, 2*pi - 2*pi/magnet_n, magnet_n);

    p_local = [magnet_r * cos(theta);
               magnet_r * sin(theta);
               zeros(1, magnet_n)];

    p_global = mtimes(R, p_local);
    P_world = [p_global(1,:) + x(1);
               p_global(2,:) + x(2);
               p_global(3,:) + x(3)];        % 3 x n

    %% Magnetic field at discretized points via Function.map
    % Build single-point field Function once, then vectorize via .map(n).
    % U_rep references the same symbolic u in every column (no extra cost).
    f_point = singlePointFieldCasADi(params);
    F_point = f_point.map(magnet_n);

    U_rep = repmat(u, 1, magnet_n);
    B = F_point(P_world, U_rep);             % 3 x n

    bx = B(1, :);
    by = B(2, :);
    bz = B(3, :);

    %% Lorentz force on levitating magnet
    K_coeff = -magnet_J / mu0;

    tangent_local = [cos(theta + pi/2);
                     sin(theta + pi/2);
                     zeros(1, magnet_n)];
    tangent_global = mtimes(R, tangent_local);

    F = crossProduct3D(K_coeff * magnet_l * tangent_global, [bx; by; bz]);

    fx = magnet_r * trapzCasADi(theta, F(1,:));
    fy = magnet_r * trapzCasADi(theta, F(2,:));
    fz = magnet_r * trapzCasADi(theta, F(3,:));

    %% Torque on levitating magnet
    nvec_local  = [zeros(2, magnet_n); ones(1, magnet_n)];
    nvec_global = mtimes(R, nvec_local);

    T = crossProduct3D(K_coeff * magnet_l * nvec_global, [bx; by; bz]);

    tx = magnet_r * trapzCasADi(theta, T(1,:));
    ty = magnet_r * trapzCasADi(theta, T(2,:));
    tz = magnet_r * trapzCasADi(theta, T(3,:));

    %% Linear acceleration
    a_linear = [fx; fy; fz] / mass - [0; 0; g];

    %% Angular acceleration (Euler's equation for rigid body)
    I_matrix = diag(I_vec);
    omega    = x(10:12);
    I_omega  = mtimes(I_matrix, omega);

    T_gyro = [omega(2)*I_omega(3) - omega(3)*I_omega(2);
              omega(3)*I_omega(1) - omega(1)*I_omega(3);
              omega(1)*I_omega(2) - omega(2)*I_omega(1)];

    alpha_ang = mtimes(inv(I_matrix), ([tx; ty; tz] - T_gyro));

    %% Assemble state derivative
    dx = [x(7:9); x(10:12); a_linear; alpha_ang];
end
