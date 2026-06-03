function S = disturbance_metrics(R, M, cfg)
% DISTURBANCE_METRICS  Recovery metrics for runs with an applied disturbance.
%
%   S = disturbance_metrics(R, M, cfg)
%
% Returns a struct with disturbance-specific fields. When the run has no
% disturbance (cfg.disturbance_schedule is empty) or the disturbance step
% was not reached, all fields are NaN so downstream tables can ignore them
% without a special case.
%
% Fields
%   .has_disturbance                logical
%   .dist_step                      step index at which the disturbance fired
%   .dist_time_s                    that step's time in seconds
%   .dist_recovery_time_ms          time from disturbance until ||x_ctrl-xEq||
%                                    decays to <= 5% of initial error; NaN if
%                                    it never recovered within the run
%   .dist_max_deviation_post_mm     max ||position - xEq_pos|| AFTER disturbance
%   .dist_max_z_deviation_post_mm   max |z_plant - z_eq| after disturbance
%   .dist_post_solve_time_spike_ms  max acados time_tot in the first 500 ms
%                                    AFTER the disturbance
%
% (Pre-disturbance metrics are available via tracking_metrics on the full
% trajectory; this module is purely about the post-event response.)

    S = nan_struct();

    if isempty(cfg.disturbance_schedule) || isnan(R.disturbance_step) ...
            || R.disturbance_step >= R.n_actual
        S.has_disturbance = false;
        return
    end
    S.has_disturbance = true;

    k = R.disturbance_step;
    S.dist_step   = k;
    S.dist_time_s = (k - 1) * R.dt;

    % --- Recovery time on controller-side error norm -----------------
    err_seq  = vecnorm(R.X_ctrl - M.xEq_ctrl);
    init_err = err_seq(1);
    if init_err <= 0
        S.dist_recovery_time_ms = NaN;
    else
        thr = 0.05 * init_err;
        recovered_at = NaN;
        for j = (k+1):numel(err_seq)
            if err_seq(j) <= thr
                recovered_at = j;
                break
            end
        end
        if isnan(recovered_at)
            S.dist_recovery_time_ms = NaN;
        else
            S.dist_recovery_time_ms = (recovered_at - k) * R.dt * 1e3;
        end
    end

    % --- Max deviation AFTER the disturbance -------------------------
    Xp = R.X_plant(:, (k+1):(R.n_actual + 1));
    pos_dev = vecnorm(Xp(1:3, :) - M.xEq_plant(1:3));
    S.dist_max_deviation_post_mm   = max(pos_dev) * 1e3;
    S.dist_max_z_deviation_post_mm = max(abs(Xp(3, :) - M.xEq_plant(3))) * 1e3;

    % --- Solve-time spike in the first 500 ms after disturbance ------
    Ts = R.dt;
    window_steps = max(1, round(0.5 / Ts));
    end_idx = min(R.n_actual, k + window_steps);
    spike_window = R.timing.tot(k:end_idx);
    spike_window = spike_window(~isnan(spike_window));
    if isempty(spike_window)
        S.dist_post_solve_time_spike_ms = NaN;
    else
        S.dist_post_solve_time_spike_ms = max(spike_window) * 1e3;
    end
end

function S = nan_struct()
    S = struct( ...
        'has_disturbance',                false, ...
        'dist_step',                      NaN, ...
        'dist_time_s',                    NaN, ...
        'dist_recovery_time_ms',          NaN, ...
        'dist_max_deviation_post_mm',     NaN, ...
        'dist_max_z_deviation_post_mm',   NaN, ...
        'dist_post_solve_time_spike_ms',  NaN ...
    );
end
