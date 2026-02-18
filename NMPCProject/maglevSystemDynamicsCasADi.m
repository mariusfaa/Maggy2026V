function dx = maglevSystemDynamicsCasADi(x, u, params)
% MAGLEVSYSTEMDYNAMICSCASADI implements the dynamics f in dxdt = f(x,u)
% for a magnetic levitation system using CasADi symbolic variables.
% This version is compatible with acados for optimal control.
%
% Inputs:
%   x - State vector (12x1 CasADi SX/MX): [position; orientation; velocity; angular_velocity]
%       x(1:3)   - [x, y, z] position of levitating magnet
%       x(4:6)   - [roll, pitch, yaw] orientation
%       x(7:9)   - [vx, vy, vz] linear velocity
%       x(10:12) - [wx, wy, wz] angular velocity
%   u - Control input (CasADi SX/MX): current through solenoids
%   params - Parameter struct (same as original implementation)
%
% Output:
%   dx - State derivative (12x1 CasADi SX/MX)
%
% Example:
%   import casadi.*
%   x = SX.sym('x', 12);
%   u = SX.sym('u', 4);
%   params = load('params.mat');
%   dx = maglevSystemDynamicsCasADi(x, u, params);
%
% Author: Adapted for CasADi compatibility
% Date: 2026

    %% Extract parameters
    m = params.magnet.m;                    % Mass of levitating magnet
    I_vec = params.magnet.I;                % Moment of inertia [Ixx, Iyy, Izz]
    g = params.physical.g;                  % Gravitational acceleration
    mu0 = params.physical.mu0;              % Magnetic permeability
    
    magnet_r = params.magnet.r;             % Radius of levitating magnet
    magnet_l = params.magnet.l;             % Length/thickness of magnet
    magnet_J = params.magnet.J;             % Current density
    magnet_n = params.magnet.n;             % Number of discretization points
    
    %% Compute rotation matrix from Euler angles
    alpha = x(4);      % Roll
    beta = x(5);    % Pitch
    gamma = x(6);      % Yaw
    
    % Rotation matrix: R = Rz(psi) * Ry(theta) * Rx(phi)
    R = computeRotationMatrix(alpha, beta, gamma);
    
    %% Discretize levitating magnet circumference
    theta_discrete = linspace(0, 2*pi - 2*pi/magnet_n, magnet_n);
    
    % Local coordinates on magnet (in magnet frame)
    px_local = magnet_r * cos(theta_discrete);
    py_local = magnet_r * sin(theta_discrete);
    pz_local = zeros(1, magnet_n);
    
    % Transform to global coordinates
    p_local = [px_local; py_local; pz_local];
    p_global = mtimes(R, p_local);
    
    % Add position offset
    px = p_global(1,:) + x(1);
    py = p_global(2,:) + x(2);
    pz = p_global(3,:) + x(3);
    
    %% Compute magnetic field at discretized points
    [bx, by, bz] = computeMagneticField(px, py, pz, u, params);
    
    %% Compute force on levitating magnet
    K = -magnet_J / mu0;
    
    % Tangent vectors at each point (perpendicular to radial direction)
    tangent_local = [cos(theta_discrete + pi/2); 
                     sin(theta_discrete + pi/2); 
                     zeros(1, magnet_n)];
    tangent_global = mtimes(R, tangent_local);
    
    % Force density: F = K * l * (tangent × B)
    F = crossProduct3D(K * magnet_l * tangent_global, [bx; by; bz]);
    
    % Integrate force around circumference using trapezoidal rule
    fx = magnet_r * trapzCasADi(theta_discrete, F(1,:));
    fy = magnet_r * trapzCasADi(theta_discrete, F(2,:));
    fz = magnet_r * trapzCasADi(theta_discrete, F(3,:));
    
    %% Compute torque on levitating magnet
    % Normal vectors (pointing up in magnet frame)
    nvec_local = [zeros(2, magnet_n); ones(1, magnet_n)];
    nvec_global = mtimes(R, nvec_local);
    
    % Torque density: T = K * l * (nvec × B)
    T = crossProduct3D(K * magnet_l * nvec_global, [bx; by; bz]);
    
    % Integrate torque around circumference
    tx = magnet_r * trapzCasADi(theta_discrete, T(1,:));
    ty = magnet_r * trapzCasADi(theta_discrete, T(2,:));
    tz = magnet_r * trapzCasADi(theta_discrete, T(3,:));
    
    %% Compute dynamics
    % Linear acceleration
    F_total = [fx; fy; fz];
    F_gravity = [0; 0; -m*g];
    a_linear = (F_total + F_gravity) / m;
    
    % Angular acceleration (considering gyroscopic effects)
    I_matrix = [I_vec(1), 0, 0; 
                0, I_vec(2), 0; 
                0, 0, I_vec(3)];
    
    omega = x(10:12);
    T_total = [tx; ty; tz];
    
    % Gyroscopic torque: omega × (I * omega)
    I_omega = mtimes(I_matrix, omega);
    T_gyro = [omega(2)*I_omega(3) - omega(3)*I_omega(2);
              omega(3)*I_omega(1) - omega(1)*I_omega(3);
              omega(1)*I_omega(2) - omega(2)*I_omega(1)];
    
    alpha = mtimes(inv(I_matrix), (T_total - T_gyro));
    
    %% State derivative
    dx = [x(7:9);           % Position derivative = velocity
          x(10:12);         % Orientation derivative = angular velocity
          a_linear;         % Velocity derivative = linear acceleration
          alpha];           % Angular velocity derivative = angular acceleration
    
end

%% Helper function: Compute rotation matrix
function R = computeRotationMatrix(alpha, beta, gamma)

    % Rotation about x-axis (roll)
    Rx = [1, 0, 0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha), cos(alpha)];
    
    % Rotation about y-axis (pitch)
    Ry = [cos(beta), 0, sin(beta);
          0, 1, 0;
          -sin(beta), 0, cos(beta)];
    
    % Rotation about z-axis (yaw)
    Rz = [cos(gamma), -sin(gamma), 0;
          sin(gamma), cos(gamma), 0;
          0, 0, 1];
    
    % Combined rotation: R = Rz * Ry * Rx
    R = mtimes(mtimes(Rz, Ry), Rx);
end

%% Helper function: Compute magnetic field from base
function [bx, by, bz] = computeMagneticField(x, y, z, u, params)

    n_points = length(x);
    
    % Initialize field components
    bx = zeros(1, n_points);
    by = zeros(1, n_points);
    bz = zeros(1, n_points);
    
    mu0 = params.physical.mu0;
    
    %% Contribution from permanent magnets
    n_perm = length(params.permanent.r);
    for i = 1:n_perm
        I_perm = params.permanent.J / mu0 * params.permanent.l(i);
        
        % Compute field from this permanent magnet
        [bx_temp, by_temp, bz_temp] = computeCircularWireField(...
            x - params.permanent.x(i), ...
            y - params.permanent.y(i), ...
            z - params.permanent.z(i), ...
            params.permanent.r(i), ...
            I_perm, ...
            mu0);
        
        bx = bx + bx_temp;
        by = by + by_temp;
        bz = bz + bz_temp;
    end
    
    %% Contribution from solenoids (controlled)
    n_sol = length(params.solenoids.r);
    for i = 1:n_sol
        I_sol = u(i);
        
        % Compute field from this solenoid
        [bx_temp, by_temp, bz_temp] = computeCircularWireField(...
            x - params.solenoids.x(i), ...
            y - params.solenoids.y(i), ...
            z - params.solenoids.z(i), ...
            params.solenoids.r(i), ...
            I_sol, ...
            mu0);
        
        % Multiply by number of windings
        bx = bx + bx_temp * params.solenoids.nw;
        by = by + by_temp * params.solenoids.nw;
        bz = bz + bz_temp * params.solenoids.nw;
    end
end

%% Helper function: Magnetic field from circular wire
function [bx, by, bz] = computeCircularWireField(x, y, z, r, I, mu0)

    % Convert to polar coordinates
    rho = sqrt(x.^2 + y.^2);
    phi = atan2(y, x);
    
    % Compute field in polar coordinates
    [bPhi, bRho, bz] = computeCircularWireFieldPolar(rho, z, r, I, mu0);
    
    % Convert back to Cartesian coordinates
    bx = bRho .* cos(phi) - bPhi .* sin(phi);
    by = bRho .* sin(phi) + bPhi .* cos(phi);
end

%% Helper function: Magnetic field in polar coordinates
function [bphi, brho, bz] = computeCircularWireFieldPolar(rho, z, r, I, mu0)

    % Handle rho ≈ 0 case with smooth transition
    tol = 1e-6;
    
    % Coefficient
    c = mu0 * I / (4 * pi * sqrt(r .* rho));
    
    % Compute k^2 parameter for elliptic integrals
    k2 = 4 * r * rho ./ ((r + rho).^2 + z.^2);
    k2 = fmin(fmax(k2, 0), 1 - 1e-9);  % Clamp to valid range
    
    % Compute elliptic integrals using CasADi-compatible version
    [K, E] = ellipke_casadi(k2);
    
    % Magnetic field components (general case, rho != 0)
    sqrt_k2 = sqrt(k2);
    denominator = (rho - r).^2 + z.^2;
    
    brho = -(z ./ rho) .* c .* sqrt_k2 .* ...
           (K - (rho.^2 + r^2 + z.^2) ./ denominator .* E);
    
    bz = c .* sqrt_k2 .* ...
         (K - (rho.^2 - r^2 + z.^2) ./ denominator .* E);
    
    % For rho ≈ 0, use analytical solution on axis
    % Use smooth transition with tanh or if_else
    axis_bz = mu0 * r^2 * I ./ (2 * (r^2 + z.^2).^(3/2));
    
    % Smooth blending (you can also use if_else for piecewise)
    weight = 0.5 * (1 + tanh((rho - tol) / (tol/10)));
    brho = brho .* weight;
    bz = bz .* weight + axis_bz .* (1 - weight);
    
    % bphi is always zero due to axial symmetry
    bphi = 0 * rho;  % Ensures correct symbolic dimension
end

%% Helper function: Cross product for 3D vectors (matrix form)
function c = crossProduct3D(a, b)
    % a and b are 3xN matrices
    % Returns 3xN matrix where each column is the cross product

    c = [a(2,:) .* b(3,:) - a(3,:) .* b(2,:);
         a(3,:) .* b(1,:) - a(1,:) .* b(3,:);
         a(1,:) .* b(2,:) - a(2,:) .* b(1,:)];
end

%% Helper function: Trapezoidal integration for CasADi
function integral = trapzCasADi(theta, y)
    % Trapezoidal rule integration compatible with CasADi
    % theta: discrete angles (numeric array, 1xN)
    % y: function values (CasADi symbolic, 1xN)
    
    % Number of points
    N = length(theta);
    
    % Spacing (uniform) 
    dtheta = 2*pi / N;  % For periodic [0, 2π) with N points
    
    % Sum along second dimension (columns) to get a scalar from 1xN row vector
    integral = dtheta * sum2(y);
end