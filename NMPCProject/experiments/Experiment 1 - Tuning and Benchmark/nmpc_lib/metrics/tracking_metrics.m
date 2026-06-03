function S = tracking_metrics(R, M, cfg)
% TRACKING_METRICS  Stabilization-quality metrics from a closed-loop run.
%
%   S = tracking_metrics(R, M, cfg)
%
% Returns:
%   .settling_1pct_ms, _2pct_ms, _5pct_ms  -- time (ms) until ||x_ctrl-xEq||
%                                              first stays <= threshold * init_err
%   .peak_pos_mm, .peak_ang_deg, .peak_yaw_deg, .peak_z_dev_mm  -- physical peaks
%   .final_err_norm                          -- ||x_ctrl(end) - xEq_ctrl||
%   .final_err_pos_mm, .final_err_ang_deg     -- final position / orientation error
%   .J_integrated                            -- integrated stage cost using
%                                              cfg's (Q, R); divergent runs get
%                                              a 1e9 penalty so they don't masquerade
%                                              as cheap.
%
% NB: physical peaks are computed on the 12-state PLANT trajectory (honest
% assessment of what actually happened in simulation), not the controller-
% side projection.

    dt  = R.dt;
    n   = R.n_actual;
    nxc = M.nx_ctrl;
    nu  = M.nu;

    Q = as_matrix(cfg.Q, nxc);
    Rm = as_matrix(cfg.R, nu);

    % --- Settling times on controller-side error norm ----------------
    Xc       = R.X_ctrl;
    err_seq  = vecnorm(Xc - M.xEq_ctrl);     % 1 x (n+1)
    init_err = err_seq(1);

    [S.settling_1pct_ms, S.settling_2pct_ms, S.settling_5pct_ms] = ...
        settle_times(err_seq, init_err, dt);

    % --- Peaks on plant trajectory -----------------------------------
    Xp = R.X_plant(:, 1:(n+1));
    pos_err = vecnorm(Xp(1:3, :) - M.xEq_plant(1:3));     % 1 x (n+1)
    S.peak_pos_mm     = max(pos_err) * 1e3;
    S.peak_ang_deg    = rad2deg(max(vecnorm(Xp(4:5, :) - M.xEq_plant(4:5))));
    S.peak_yaw_deg    = rad2deg(max(abs(Xp(6, :) - M.xEq_plant(6))));
    S.peak_z_dev_mm   = max(abs(Xp(3, :) - M.xEq_plant(3))) * 1e3;

    % --- Final-error summary -----------------------------------------
    S.final_err_norm   = err_seq(end);
    S.final_err_pos_mm = norm(Xp(1:3, end) - M.xEq_plant(1:3)) * 1e3;
    S.final_err_ang_deg = rad2deg(norm(Xp(4:5, end) - M.xEq_plant(4:5)));

    % --- Integrated cost (using the run's own Q, R) ------------------
    %
    % Note: this is NOT a normalized comparison metric across configs --
    % it uses the cfg's weights. For cross-config comparison, callers
    % should also compute a J_integrated_ref using a fixed reference
    % weighting (Stage analysis layer adds this).
    Xerr = R.X_ctrl(:, 1:n) - M.xEq_ctrl;
    Uerr = R.U          - M.uEq;
    Jx = sum(Xerr .* (Q  * Xerr), 1);   % 1 x n
    Ju = sum(Uerr .* (Rm * Uerr), 1);
    J  = (sum(Jx) + sum(Ju)) * dt;
    if R.diverged
        J = J + 1e9;
    end
    S.J_integrated = J;
end

% ---------------------------------------------------------------------- %

function [s1, s2, s5] = settle_times(err_seq, init_err, dt)
    s1 = NaN; s2 = NaN; s5 = NaN;
    if init_err <= 0; return; end
    thr = init_err * [0.01, 0.02, 0.05];
    s = nan(1, 3);
    for k = 1:3
        for j = 1:numel(err_seq)
            if all(err_seq(j:end) <= thr(k))
                s(k) = (j - 1) * dt;
                break
            end
        end
    end
    s1 = s(1) * 1e3;
    s2 = s(2) * 1e3;
    s5 = s(3) * 1e3;
end

function W = as_matrix(W_in, n)
    if isvector(W_in); W = diag(W_in(:)); else; W = W_in; end
    if any(size(W) ~= [n n])
        error('tracking_metrics:bad_weight', ...
              'Weight matrix has size %dx%d, expected %dx%d', ...
              size(W,1), size(W,2), n, n);
    end
end
