function dx = maglevSystemDynamicsCasADi(x, u, params)
% MAGLEVSYSTEMDYNAMICSCASADI  Dynamics f in dx/dt = f(x, u) for a magnetic
% levitation system, formulated with CasADi symbolic variables for use
% with acados optimal control.
%
% This version uses the small-angle simplification dx(4:6) = x(10:12),
% i.e., Euler angle rates equal body angular velocities. This is the model
% around which equilibria were computed.
%
% Inputs:
%   x      - State vector (12x1 CasADi SX/MX):
%              x(1:3)   [x, y, z]           position of levitating magnet
%              x(4:6)   [roll, pitch, yaw]   orientation (Euler angles)
%              x(7:9)   [vx, vy, vz]         linear velocity
%              x(10:12) [wx, wy, wz]         angular velocity (body frame)
%   u      - Control input (CasADi SX/MX): solenoid currents
%   params - Parameter struct (see documentation)
%
% Output:
%   dx     - State derivative (12x1 CasADi SX/MX)
%
% See also: computeRotationMatrixCasADi, computeMagneticFieldCasADi

    import casadi.*

    %% Extract parameters
    mass   = params.magnet.m;
    I_vec  = params.magnet.I;          % [Ixx, Iyy, Izz]
    g      = params.physical.g;
    mu0    = params.physical.mu0;

    magnet_r = params.magnet.r;
    magnet_l = params.magnet.l;
    magnet_J = params.magnet.J;
    magnet_n = params.magnet.n;

    %% Rotation matrix from Euler angles
    R = computeRotationMatrixCasADi(x(4), x(5), x(6));

    %% Discretize levitating magnet circumference
    theta = linspace(0, 2*pi - 2*pi/magnet_n, magnet_n);

    p_local = [magnet_r * cos(theta);
               magnet_r * sin(theta);
               zeros(1, magnet_n)];

    p_global = mtimes(R, p_local);
    px = p_global(1,:) + x(1);
    py = p_global(2,:) + x(2);
    pz = p_global(3,:) + x(3);

    %% Magnetic field at discretized points
    [bx, by, bz] = computeMagneticFieldCasADi(px, py, pz, u, params);

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
    %   dx(1:3)   = velocity
    %   dx(4:6)   = angular velocity  (small-angle approximation)
    %   dx(7:9)   = linear acceleration
    %   dx(10:12) = angular acceleration
    dx = [x(7:9); x(10:12); a_linear; alpha_ang];
end