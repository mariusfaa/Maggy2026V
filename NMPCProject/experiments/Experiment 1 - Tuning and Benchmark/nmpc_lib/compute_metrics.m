function S = compute_metrics(R, M, cfg, Q_ref, R_ref)
% COMPUTE_METRICS  Reduce a single closed-loop run to a summary struct.
%
%   S = compute_metrics(R, M, cfg, Q_ref, R_ref)
%
% Inputs
%   R         struct from run_sim (one repetition)
%   M         struct from build_model
%   cfg       cfg used for the run (.lbx_idx_zero_based, .lbx, .ubx for
%             violation tracking)
%   Q_ref     reference Q on x_ctrl for J_integrated (always Bryson Q so
%             cross-config comparison is well-defined)
%   R_ref     reference R on u (always identity)
%
% Output S — fields used downstream by the post-processor:
%   .n_actual, .diverged, .all_converged
%   .t_mean_ms, .t_p95_ms, .t_p99_ms, .t_max_ms       (acados time_tot)
%   .t_wall_mean_ms, .t_wall_p99_ms, .t_wall_max_ms   (wall-clock)
%   .J_integrated                                      (using Q_ref, R_ref)
%   .settling_1pct_ms, .settling_2pct_ms, .settling_5pct_ms
%   .peak_pos_mm, .peak_ang_deg, .peak_yaw_deg, .peak_z_dev_mm
%   .control_effort, .pct_saturated
%   .n_hard_viol, .max_hard_viol
%   .sqp_iter_mean, .qp_iter_mean
%   .time_lin_mean_ms, .time_qp_sol_mean_ms, .time_reg_mean_ms

    n   = R.n_actual;
    nu  = M.nu;
    nxc = M.nx_ctrl;
    dt  = R.dt;

    if isvector(Q_ref); Q_ref = diag(Q_ref(:)); end
    if isvector(R_ref); R_ref = diag(R_ref(:)); end

    % --- timing (only valid steps) ---
    tv     = R.timing.tot(1:n);
    tw     = R.timing.wall(1:n);
    t_lin  = R.timing.lin(1:n);
    t_qp   = R.timing.qp_sol(1:n);
    t_reg  = R.timing.reg(1:n);

    S.t_mean_ms      = mean(tv) * 1e3;
    S.t_p95_ms       = pctile(tv, 95) * 1e3;
    S.t_p99_ms       = pctile(tv, 99) * 1e3;
    S.t_max_ms       = max(tv) * 1e3;
    S.t_wall_mean_ms = mean(tw) * 1e3;
    S.t_wall_p99_ms  = pctile(tw, 99) * 1e3;
    S.t_wall_max_ms  = max(tw) * 1e3;

    S.time_lin_mean_ms    = mean(t_lin) * 1e3;
    S.time_qp_sol_mean_ms = mean(t_qp)  * 1e3;
    S.time_reg_mean_ms    = mean(t_reg) * 1e3;

    % --- iteration counts ---
    S.sqp_iter_mean = mean(R.sqp_iter(1:n));
    qv = R.qp_iter(1:n);
    S.qp_iter_mean  = nanmean_(qv);

    % --- divergence / convergence ---
    S.diverged      = R.diverged;
    S.n_actual      = n;
    S.all_converged = all(R.status(1:n) == 0);
    S.pct_status_0  = 100 * sum(R.status(1:n) == 0) / max(n, 1);

    % --- closed-loop integrated stage cost (using reference Q, R) ---
    %   J = sum_i (x_i' Q_ref x_i + u_i' R_ref u_i) * dt
    %   x relative to xEq_ctrl, u relative to uEq.
    Xc = R.X_ctrl(:, 1:n);                 % nxc x n
    Uc = R.U(:, 1:n);                       % nu x n
    Xerr = Xc - M.xEq_ctrl;
    Uerr = Uc - M.uEq;
    Jx = sum(Xerr .* (Q_ref * Xerr), 1);    % 1 x n
    Ju = sum(Uerr .* (R_ref * Uerr), 1);
    if R.diverged
        % Penalize divergent runs heavily so they don't masquerade as cheap.
        S.J_integrated = (sum(Jx) + sum(Ju)) * dt + 1e9;
    else
        S.J_integrated = (sum(Jx) + sum(Ju)) * dt;
    end

    % --- physical peaks (use 12-state plant trajectory for honesty) ---
    Xp = R.X_plant(:, 1:n+1);
    pos_err = vecnorm(Xp(1:3, :) - M.xEq_plant(1:3));     % 1 x (n+1)
    S.peak_pos_mm = max(pos_err) * 1e3;
    ang_err = vecnorm(Xp(4:5, :) - M.xEq_plant(4:5));
    S.peak_ang_deg = rad2deg(max(ang_err));
    S.peak_yaw_deg = rad2deg(max(abs(Xp(6, :) - M.xEq_plant(6))));
    S.peak_z_dev_mm = max(abs(Xp(3, :) - M.xEq_plant(3))) * 1e3;

    % --- control effort / saturation ---
    S.control_effort = sum(sum(Uc.^2, 1)) * dt;
    if n > 0
        S.pct_saturated = 100 * sum(any(abs(Uc) > 0.999, 1)) / n;
    else
        S.pct_saturated = 0;
    end

    % --- hard state-box violations on the constrained controller-side states ---
    % cfg.lbx_idx_zero_based is 0-based for acados; convert to 1-based MATLAB
    if isfield(cfg, 'lbx_idx_zero_based') && ~isempty(cfg.lbx_idx_zero_based)
        idx1 = cfg.lbx_idx_zero_based + 1;
        n_v = 0; max_v = 0;
        for j = 1:size(R.X_ctrl, 2)
            if j > n+1; break; end
            for c = 1:length(idx1)
                v = max([cfg.lbx(c) - R.X_ctrl(idx1(c), j), ...
                         R.X_ctrl(idx1(c), j) - cfg.ubx(c), 0]);
                if v > 0
                    n_v = n_v + 1;
                    max_v = max(max_v, v);
                end
            end
        end
        S.n_hard_viol   = n_v;
        S.max_hard_viol = max_v;
    else
        S.n_hard_viol   = 0;
        S.max_hard_viol = 0;
    end

    % --- settling times based on ||x_ctrl - xEq_ctrl|| ---
    % Threshold at 1%, 2%, 5% of the initial error norm.
    err_seq = vecnorm(R.X_ctrl(:, 1:n+1) - M.xEq_ctrl);
    init_err = err_seq(1);
    thr_pct  = [0.01, 0.02, 0.05];
    settling = nan(size(thr_pct));
    if init_err > 0
        for s = 1:length(thr_pct)
            thr = thr_pct(s) * init_err;
            for j = 1:length(err_seq)
                if all(err_seq(j:end) <= thr)
                    settling(s) = (j - 1) * dt;
                    break;
                end
            end
        end
    end
    S.settling_1pct_ms = settling(1) * 1e3;
    S.settling_2pct_ms = settling(2) * 1e3;
    S.settling_5pct_ms = settling(3) * 1e3;
end

function p = pctile(x, q)
    x = sort(x(~isnan(x)));
    n = length(x);
    if n == 0; p = NaN; return; end
    r = (q / 100) * n;
    lo = max(floor(r), 1);
    hi = min(ceil(r), n);
    if lo == hi
        p = x(lo);
    else
        p = x(lo) + (r - lo) * (x(hi) - x(lo));
    end
end

function m = nanmean_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = mean(x); end
end
