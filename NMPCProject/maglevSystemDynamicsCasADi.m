function dx = maglevSystemDynamicsCasADi(x, u, params)
% MAGLEVSYSTEMDYNAMICSCASADI implements the dynamics f in dxdt = f(x,u)
% for a magnetic levitation system using CasADi symbolic variables.
% This version is compatible with acados for optimal control.
%
% This version exactly matches the original maglevSystemDynamics.m, which
% uses the simplification dx(4:6) = x(10:12), i.e., Euler angle rates
% equal body angular velocities. This is valid for small angles and is
% the model around which equilibria were computed.
%
% Inputs:
%   x - State vector (12x1 CasADi SX/MX): [position; orientation; velocity; angular_velocity]
%       x(1:3)   - [x, y, z] position of levitating magnet
%       x(4:6)   - [roll, pitch, yaw] orientation (Euler angles)
%       x(7:9)   - [vx, vy, vz] linear velocity
%       x(10:12) - [wx, wy, wz] angular velocity (body frame)
%   u - Control input (CasADi SX/MX): current through solenoids
%   params - Parameter struct (same as original implementation)
%
% Output:
%   dx - State derivative (12x1 CasADi SX/MX)

    import casadi.*
    
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
    alpha = x(4);   % Roll
    beta = x(5);    % Pitch
    gamma = x(6);   % Yaw
    
    % Rotation matrix: R = Rz(gamma) * Ry(beta) * Rx(alpha)
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
    
    % Force density: F = K * l * (tangent x B)
    F = crossProduct3D(K * magnet_l * tangent_global, [bx; by; bz]);
    
    % Integrate force around circumference using trapezoidal rule
    fx = magnet_r * trapzCasADi(theta_discrete, F(1,:));
    fy = magnet_r * trapzCasADi(theta_discrete, F(2,:));
    fz = magnet_r * trapzCasADi(theta_discrete, F(3,:));
    
    %% Compute torque on levitating magnet
    % Normal vectors (pointing up in magnet frame)
    nvec_local = [zeros(2, magnet_n); ones(1, magnet_n)];
    nvec_global = mtimes(R, nvec_local);
    
    % Torque density: T = K * l * (nvec x B)
    T = crossProduct3D(K * magnet_l * nvec_global, [bx; by; bz]);
    
    % Integrate torque around circumference
    tx = magnet_r * trapzCasADi(theta_discrete, T(1,:));
    ty = magnet_r * trapzCasADi(theta_discrete, T(2,:));
    tz = magnet_r * trapzCasADi(theta_discrete, T(3,:));
    
    %% Compute dynamics — matching original maglevSystemDynamics.m exactly
    % Original computes:
    %   f = M\([fx;fy;fz;tx;ty;tz] - [0;0;0;cross(omega,I*omega)]) - [0;0;g;0;0;0]
    %   dx = A*x + B*f
    % which expands to:
    %   dx(1:6)  = x(7:12)        (positions/angles from velocities)
    %   dx(7:12) = f               (accelerations)
    
    % Linear acceleration: f(1:3) = [fx;fy;fz]/m - [0;0;g]
    a_linear = [fx; fy; fz] / m - [0; 0; g];
    
    % Angular acceleration: f(4:6) = inv(I)*(tau - omega x I*omega)
    I_matrix = [I_vec(1), 0, 0; 
                0, I_vec(2), 0; 
                0, 0, I_vec(3)];
    
    omega = x(10:12);
    T_total = [tx; ty; tz];
    
    % Gyroscopic torque: omega x (I * omega)
    I_omega = mtimes(I_matrix, omega);
    T_gyro = [omega(2)*I_omega(3) - omega(3)*I_omega(2);
              omega(3)*I_omega(1) - omega(1)*I_omega(3);
              omega(1)*I_omega(2) - omega(2)*I_omega(1)];
    
    alpha_ang = mtimes(inv(I_matrix), (T_total - T_gyro));
    
    %% State derivative — matching original: dx = A*x + B*f
    % dx(1:3)   = x(7:9)      velocity
    % dx(4:6)   = x(10:12)    angular velocity (NOT Euler rate transform)
    % dx(7:9)   = a_linear     linear acceleration
    % dx(10:12) = alpha_ang    angular acceleration
    dx = [x(7:9);       % Position derivative = velocity
          x(10:12);     % Orientation derivative = angular velocity (matches original)
          a_linear;     % Velocity derivative = linear acceleration
          alpha_ang];   % Angular velocity derivative = angular acceleration
    
end

%% Helper function: Compute rotation matrix
function R = computeRotationMatrix(alpha, beta, gamma)
    import casadi.*
    
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
    import casadi.*
    
    n_points = length(x);
    
    % Initialize field components
    bx = SX.zeros(1, n_points);
    by = SX.zeros(1, n_points);
    bz = SX.zeros(1, n_points);
    
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
    import casadi.*
    
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
    import casadi.*

    tol = 1e-6;

    % Guard rho to prevent division by zero in c and z./rho
    % This only affects the general formula; if_else will discard it on-axis
    rho_safe = fmax(rho, tol);

    % Coefficient (using rho_safe to avoid Inf)
    c = mu0 * I ./ (4 * pi * sqrt(r .* rho_safe));

    % k^2 parameter for elliptic integrals
    k2 = 4 * r * rho_safe ./ ((r + rho_safe).^2 + z.^2);
    k2 = fmin(fmax(k2, 0), 1 - 1e-9);

    % Elliptic integrals
    [K, E] = ellipke_casadi(k2);

    sqrt_k2 = sqrt(k2);
    denominator = (rho_safe - r).^2 + z.^2;

    % General formula (valid away from axis)
    brho_general = -(z ./ rho_safe) .* c .* sqrt_k2 .* ...
                   (K - (rho_safe.^2 + r^2 + z.^2) ./ denominator .* E);

    bz_general   =  c .* sqrt_k2 .* ...
                   (K - (rho_safe.^2 - r^2 + z.^2) ./ denominator .* E);

    % On-axis formula (exact, valid only at rho = 0)
    bz_axis = mu0 * r^2 * I ./ (2 * (r^2 + z.^2).^(3/2));

    % Select formula based on rho, mimicking the original's hard switch
    brho = if_else(rho < tol, 0,       brho_general);
    bz   = if_else(rho < tol, bz_axis, bz_general);

    % bphi is zero due to axial symmetry
    bphi = 0 * rho;
end

%% Helper function: Cross product for 3D vectors (matrix form)
function c = crossProduct3D(a, b)
    % a and b are 3xN matrices
    % Returns 3xN matrix where each column is the cross product
    import casadi.*
    
    c = [a(2,:) .* b(3,:) - a(3,:) .* b(2,:);
         a(3,:) .* b(1,:) - a(1,:) .* b(3,:);
         a(1,:) .* b(2,:) - a(2,:) .* b(1,:)];
end

%% Helper function: Trapezoidal integration for CasADi
function integral = trapzCasADi(theta, y)
    % Trapezoidal rule integration compatible with CasADi
    % theta: discrete angles (numeric array, 1xN)
    % y: function values (CasADi symbolic, 1xN)
    import casadi.*
    
    % Close the loop by appending first point at 2*pi
    y_closed = [y, y(1)];             % 1 X (N+1)
    dtheta_vec = diff([theta, 2*pi]); % spacing for each trapezoid, 1xN
    integral = sum(0.5 * (y_closed(1:end-1) + y_closed(2:end)) .* dtheta_vec);
end