function S = timing_stats(R, cfg)
% TIMING_STATS  Solve-time statistics from a closed-loop run.
%
%   S = timing_stats(R, cfg)
%
% Returns mean / median / p90 / p95 / p99 / max / IQR jitter for BOTH the
% acados-reported time_tot and the wall-clock measurement. Embedded
% headroom = Ts*1000 - t_p99_ms (negative means real-time infeasible at
% the run's Ts).
%
% Per-component means (lin / qp_sol / sim / reg) are included for
% attribution plots in Stage C and Stage H.
%
% All times are in MILLISECONDS in S.

    Ts = cfg.Tf / cfg.N;

    S = struct();

    [S.t_mean_ms,   S.t_median_ms, S.t_p90_ms, S.t_p95_ms, S.t_p99_ms, S.t_max_ms, S.t_iqr_ms] = ...
        stats_(R.timing.tot);
    [S.t_wall_mean_ms, S.t_wall_median_ms, S.t_wall_p90_ms, S.t_wall_p95_ms, ...
     S.t_wall_p99_ms,  S.t_wall_max_ms,    S.t_wall_iqr_ms] = ...
        stats_(R.timing.wall);

    S.embedded_headroom_ms = Ts * 1e3 - S.t_p99_ms;
    S.embedded_feasible_p99 = S.embedded_headroom_ms > 0;
    S.embedded_feasible_max = (Ts * 1e3 - S.t_max_ms) > 0;

    S.time_lin_mean_ms    = mean_(R.timing.lin)    * 1e3;
    S.time_qp_sol_mean_ms = mean_(R.timing.qp_sol) * 1e3;
    S.time_sim_mean_ms    = mean_(R.timing.sim)    * 1e3;
    S.time_reg_mean_ms    = mean_(R.timing.reg)    * 1e3;
    S.time_glob_mean_ms   = mean_(R.timing.glob)   * 1e3;
end

function [m, md, p90, p95, p99, mx, iqr_] = stats_(t)
    t = t(:);
    t = t(~isnan(t));
    if isempty(t)
        m = NaN; md = NaN; p90 = NaN; p95 = NaN; p99 = NaN; mx = NaN; iqr_ = NaN;
        return
    end
    t_ms = t * 1e3;
    m   = mean(t_ms);
    md  = median(t_ms);
    p90 = pctile_(t_ms, 90);
    p95 = pctile_(t_ms, 95);
    p99 = pctile_(t_ms, 99);
    mx  = max(t_ms);
    iqr_ = pctile_(t_ms, 75) - pctile_(t_ms, 25);
end

function m = mean_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = mean(x); end
end

function p = pctile_(x, q)
    x = sort(x(~isnan(x)));
    n = length(x);
    if n == 0; p = NaN; return; end
    r = (q / 100) * n;
    lo = max(floor(r), 1);
    hi = min(ceil(r), n);
    if lo == hi; p = x(lo);
    else;        p = x(lo) + (r - lo) * (x(hi) - x(lo)); end
end
