function [Q, R, Q_legacy] = bryson_weights(model_kind)
% BRYSON_WEIGHTS  Bryson's-rule diagonal Q (and identity R) for the maglev.
%
%   [Q, R, Q_legacy] = bryson_weights('reduced10')
%   [Q, R, Q_legacy] = bryson_weights('full12')
%
% Normalizers (rationale documented in plan):
%   x_max  = [0.025, 0.025, 0.020] m       (xy radial / z half-range)
%   ang    = [0.35, 0.35] rad               (roll, pitch box)
%   v_max  = [0.5, 0.5, 0.5] m/s            (proposed; not currently constrained)
%   w_max  = [3.5, 3.5] rad/s
%   yaw    = 0.35 rad, w_yaw = 3.5 rad/s    (12-state only; weighted lightly)
%   u_max  = 1
%
% Outputs
%   Q          diagonal weight matrix on x_ctrl
%   R          identity 4x4 (R = u_max^-2 * I = I)
%   Q_legacy   the legacy hand-tuned diagonal for comparison

    q_pos = 1 ./ ([0.025, 0.025, 0.020]).^2;     % [1600, 1600, 2500]
    q_ang = 1 ./ ([0.35, 0.35]).^2;              % [8.16, 8.16]
    q_yaw = 0.01 * q_ang(1);                      % light: 0.0816
    q_vel = 1 ./ ([0.5, 0.5, 0.5]).^2;           % [4, 4, 4]
    q_w   = 1 ./ ([3.5, 3.5]).^2;                 % [0.0816, 0.0816]
    q_w_yaw = 0.01 * q_w(1);                      % light: 0.000816

    switch lower(model_kind)
        case 'reduced10'
            % Order: [x y z roll pitch vx vy vz wx wy]
            Q = diag([q_pos, q_ang, q_vel, q_w]);
            Q_legacy = diag([1e2 1e2 1e3 1e3 1e3 1e1 1e1 1e1 1e1 1e1]);
        case 'full12'
            % Order: [x y z roll pitch yaw vx vy vz wx wy wz]
            Q = diag([q_pos, q_ang, q_yaw, q_vel, q_w, q_w_yaw]);
            Q_legacy = diag([1e2 1e2 1e3 1e3 1e3 1e1 1e1 1e1 1e1 1e1 1e1 1e1]);
        otherwise
            error('bryson_weights:bad_kind', 'Unknown model_kind ''%s''.', model_kind);
    end

    R = eye(4);
end
