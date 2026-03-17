function dx = maglevSystemDynamicsReduced_casadi(x, u, params, modelId)
% MAGLEVSYSTEMDYNAMICSREDUCED_CASADI  Reduced 10-state dynamics (no yaw).
%
% State (10x1): [x; y; z; roll; pitch; vx; vy; vz; wx; wy]
%
% Yaw and its derivative are removed. With wz=0 the gyroscopic torque
% omega x (I*omega) has only a z-component, so it vanishes from the
% remaining (roll, pitch) equations.
%
% Inputs:
%   x       - State vector (10x1, CasADi SX/MX)
%   u       - Solenoid currents (n_sol x 1)
%   params  - Parameter struct
%   modelId - MaglevModel.Fast or MaglevModel.Accurate

    import casadi.*

    m     = params.magnet.m;
    I_vec = params.magnet.I;   % [Ix; Iy; Iz]
    g     = params.physical.g;

    %% Force and torque (uses x(1:3) for pos, x(4:5) for roll/pitch)
    [fx, fy, fz, tx, ty, ~] = computeForceAndTorque_casadi(x, u, params, modelId);

    %% Reduced dynamics — direct scalar form
    dx = [x(6);            % dx/dt  = vx
          x(7);            % dy/dt  = vy
          x(8);            % dz/dt  = vz
          x(9);            % droll/dt  = wx   (small-angle approx)
          x(10);           % dpitch/dt = wy
          fx / m;          % dvx/dt
          fy / m;          % dvy/dt
          fz / m - g;      % dvz/dt
          tx / I_vec(1);   % dwx/dt
          ty / I_vec(2)];  % dwy/dt
end
