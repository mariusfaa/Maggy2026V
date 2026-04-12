function dx = maglevSystemDynamics_casadi(x, u, params, modelId)
% MAGLEVSYSTEMDYNAMICS_CASADI implements dxdt = f(x,u) for the magnetic
% levitation system using CasADi symbolic variables.
%
% Inputs:
%   x       - State vector (12x1, CasADi MX)
%   u       - Solenoid currents (n_sol x 1, CasADi MX)
%   params  - Parameter struct with params.luts field
%   modelId - MaglevModel.Accurate (default) or MaglevModel.Fast
%
% Output:
%   dx      - State derivative (12x1, CasADi MX)

    import casadi.*

    %% Extract parameters
    m     = params.magnet.m;
    I_vec = params.magnet.I;
    g     = params.physical.g;

    %% Compute force and torque
    [fx, fy, fz, tx, ty, tz] = computeForceAndTorque_casadi(x, u, params, modelId);

    %% System matrices (same structure as maglevSystemDynamics.m)
    A = [zeros(6), eye(6);
         zeros(6), zeros(6)];

    B = [zeros(6);
         eye(6)];

    M = [m*eye(3),    zeros(3);
         zeros(3), diag(I_vec)];

    %% Nonlinear function f(x,u)
    omega = x(10:12);
    I_omega = [I_vec(1)*omega(1); I_vec(2)*omega(2); I_vec(3)*omega(3)];

    % Gyroscopic torque: omega x (I * omega)
    T_gyro = [omega(2)*I_omega(3) - omega(3)*I_omega(2);
              omega(3)*I_omega(1) - omega(1)*I_omega(3);
              omega(1)*I_omega(2) - omega(2)*I_omega(1)];

    f = mtimes(inv(M), ([fx; fy; fz; tx; ty; tz] - [zeros(3,1); T_gyro])) ...
        - [0; 0; g; 0; 0; 0];

    dx = mtimes(A, x) + mtimes(B, f);
end
