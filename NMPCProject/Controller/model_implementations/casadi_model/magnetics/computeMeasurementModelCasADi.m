function [h_expr, h_func] = computeMeasurementModelCasADi(x_r, i_meas_sym, params)
% COMPUTEMEASUREMENTMODELCASADI  CasADi measurement model h(x_r, i_meas)
% for the MagLev v4.3 hardware with a single 3-axis magnetic sensor.
%
% Computes the total magnetic field [Bx; By; Bz] (in Tesla) at the first
% sensor position in params.sensors, given the 10-state reduced-order
% state and measured solenoid currents.
%
% The total field is the superposition of:
%   1. Permanent base magnets  (state-independent)
%   2. Solenoids               (depends on i_meas)
%   3. Levitating magnet       (depends on x_r: position & orientation)
%
% Inputs:
%   x_r         - Reduced-order state (10x1 CasADi SX):
%                   x_r(1:3)  = [x, y, z]       position
%                   x_r(4:5)  = [alpha, beta]    roll, pitch
%                   x_r(6:8)  = [vx, vy, vz]     (not used in h)
%                   x_r(9:10) = [wx, wy]          (not used in h)
%   i_meas_sym  - Measured solenoid currents (4x1 CasADi SX), in Amps
%   params      - Parameter struct (see parameters_maggy_V4.m)
%
% Outputs:
%   h_expr      - CasADi SX expression for [Bx; By; Bz] (3x1, Tesla)
%   h_func      - CasADi Function: h_func(x_r, i_meas) -> y (3x1)
%
% See also: computeMagneticFieldCasADi, computeCircularWireFieldCasADi,
%           computeRotationMatrixCasADi

    import casadi.*

    %% Sensor position (v4.3: single sensor, use first entry)
    % Cast to CasADi SX so that all downstream computations (fmax, fmin,
    % if_else in the elliptic-integral code) receive at least one symbolic
    % argument and do not fall back to plain MATLAB doubles.
    sx = SX(params.sensors.x(1));
    sy = SX(params.sensors.y(1));
    sz = SX(params.sensors.z(1));

    %% 1. Base field at sensor (permanent magnets + solenoids)
    % computeMagneticFieldCasADi computes field from all permanent magnets
    % and all solenoids at the given query point.
    [bx_base, by_base, bz_base] = computeMagneticFieldCasADi( ...
        sx, sy, sz, i_meas_sym, params);

    %% 2. Levitating magnet's field at the sensor
    % Rotation matrix from reduced-order state (gamma = 0)
    R = computeRotationMatrixCasADi(x_r(4), x_r(5), 0);

    % Transform: sensor position relative to magnet, in magnet-local frame
    % Following computeFieldTotal.m line 32: pRotated = R^{-1} * (eta(1:3) - sensor_pos)
    p_rel = mtimes(R', x_r(1:3) - [sx; sy; sz]);

    % Equivalent current for the levitating magnet dipole (cast to SX to
    % keep the entire computation graph symbolic)
    I_lev = SX(-params.magnet.J / params.physical.mu0 * params.magnet.l / 2);

    % Magnetic field in magnet-local frame
    [bx_lev_local, by_lev_local, bz_lev_local] = computeCircularWireFieldCasADi( ...
        p_rel(1), p_rel(2), p_rel(3), ...
        params.magnet.r, I_lev, params.physical.mu0);

    % Rotate field back to world frame
    B_lev_world = mtimes(R, [bx_lev_local; by_lev_local; bz_lev_local]);

    %% 3. Total field at sensor
    h_expr = [bx_base + B_lev_world(1);
              by_base + B_lev_world(2);
              bz_base + B_lev_world(3)];

    %% Build CasADi Function
    h_func = casadi.Function('h_meas', {x_r, i_meas_sym}, {h_expr}, ...
        {'x_r', 'i_meas'}, {'y'});
end
