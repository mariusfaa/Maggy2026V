function dx = maglevSystemDynamicsDipole_casadi(x, u, params)
% MAGLEVSYSTEMDYNAMICSDIPOLE_CASADI  Reduced 10-state dynamics (dipole model).
%
% State (10x1): [x; y; z; roll; pitch; vx; vy; vz; wx; wy]
%
% Uses the magnetic dipole approximation for source fields with surface
% integration on the levitating magnet. Optionally applies a Jacobian
% gradient correction (params.dipole_corr) to match the Accurate model's
% linearization at equilibrium.
%
% Inputs:
%   x      - State vector (10x1, CasADi MX)
%   u      - Solenoid currents (4x1, MX)
%   params - Parameter struct (with optional dipole_corr field)

    m     = params.magnet.m;
    I_vec = params.magnet.I;   % [Ix; Iy; Iz]
    g     = params.physical.g;

    % Force and torque via dipole model
    [fx, fy, fz, tx, ty, ~] = computeForceAndTorque_dipole_casadi(x, u, params);

    % Jacobian correction: F += K * [x;y;z-zEq;roll;pitch] + B_corr * u
    if isfield(params, 'dipole_corr')
        K      = params.dipole_corr.K;       % 5x5 state correction
        B_corr = params.dipole_corr.B_corr;  % 5x4 input correction
        zEq    = params.dipole_corr.zEq;
        delta_q = [x(1); x(2); x(3) - zEq; x(4); x(5)];
        corr = K * delta_q + B_corr * u;
        fx = fx + corr(1);
        fy = fy + corr(2);
        fz = fz + corr(3);
        tx = tx + corr(4);
        ty = ty + corr(5);
    end

    % Reduced dynamics (same structure as non-dipole model)
    dx = [x(6);            % dx/dt  = vx
          x(7);            % dy/dt  = vy
          x(8);            % dz/dt  = vz
          x(9);            % droll/dt  = wx
          x(10);           % dpitch/dt = wy
          fx / m;          % dvx/dt
          fy / m;          % dvy/dt
          fz / m - g;      % dvz/dt
          tx / I_vec(1);   % dwx/dt
          ty / I_vec(2)];  % dwy/dt
end
