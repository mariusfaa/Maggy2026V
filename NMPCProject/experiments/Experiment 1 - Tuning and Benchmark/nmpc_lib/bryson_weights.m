function [Q, R, Q_bryson] = bryson_weights(model_kind)
% BRYSON_WEIGHTS  Reference cost weights for the maglev experiments.
%
%   [Q, R, Q_bryson] = bryson_weights('reduced10')
%   [Q, R, Q_bryson] = bryson_weights('full12')
%
% NOTE on the chosen Q (thesis-relevant): A pure Bryson's-rule normalisation
% based on the hard state-constraint bounds (±0.025 m, ±0.35 rad, ±0.5 m/s,
% ±3.5 rad/s) under-weights orientation by ~100× relative to position. For
% this open-loop-unstable maglev plant, a controller using those weights
% produces optimal OCP solves but the closed-loop trajectory diverges (the
% magnet tips over because orientation is barely penalised). We therefore
% adopt the existing hand-tuned weights from the legacy NMPC scripts as the
% baseline Q — they are demonstrably sufficient to stabilise the plant
% under the reference IRK SQP_RTI PARTIAL_HPIPM GAUSS_NEWTON setup. The
% pure-Bryson values are returned as Q_bryson for completeness and for
% the thesis to discuss the Bryson-vs-hand-tuned comparison.
%
% Outputs
%   Q          baseline diagonal weight matrix on x_ctrl (legacy hand-tuned)
%   R          identity 4×4
%   Q_bryson   pure-Bryson values for comparison/reference

    % --- Pure Bryson values (returned as Q_bryson; NOT used as primary) ---
    q_pos_b = 1 ./ ([0.025, 0.025, 0.020]).^2;     % [1600, 1600, 2500]
    q_ang_b = 1 ./ ([0.35, 0.35]).^2;              % [8.16, 8.16]
    q_yaw_b = 0.01 * q_ang_b(1);                    % light: 0.0816
    q_vel_b = 1 ./ ([0.5, 0.5, 0.5]).^2;           % [4, 4, 4]
    q_w_b   = 1 ./ ([3.5, 3.5]).^2;                 % [0.0816, 0.0816]
    q_w_yaw_b = 0.01 * q_w_b(1);                    % light: 0.000816

    switch lower(model_kind)
        case 'reduced10'
            % Order: [x y z roll pitch vx vy vz wx wy]
            Q        = diag([1e2 1e2 1e3 1e3 1e3 1e1 1e1 1e1 1e1 1e1]);
            Q_bryson = diag([q_pos_b, q_ang_b, q_vel_b, q_w_b]);
        case 'full12'
            % Order: [x y z roll pitch yaw vx vy vz wx wy wz]
            % Match the validated workingSimulator.m weights exactly:
            % q_yaw = 1e1 (10x smaller than roll/pitch), q_wz = 1e0
            % (10x smaller than wx/wy). These moderate weights keep
            % the Hessian well-conditioned and don't try to fight the
            % near-uncontrollable yaw mode.
            Q        = diag([1e2 1e2 1e3 1e3 1e3 1e1 1e1 1e1 1e1 1e1 1e1 1e0]);
            Q_bryson = diag([q_pos_b, q_ang_b, q_yaw_b, q_vel_b, q_w_b, q_w_yaw_b]);
        otherwise
            error('bryson_weights:bad_kind', 'Unknown model_kind ''%s''.', model_kind);
    end

    R = eye(4);
end
