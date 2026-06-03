function S = convergence_metrics(R)
% CONVERGENCE_METRICS  NLP/QP iteration and status statistics.
%
%   S = convergence_metrics(R)
%
% Fields
%   .sqp_iter_mean, _p95, _max
%   .qp_iter_mean,  _p95, _max
%   .pct_status_0          fraction of steps with acados status 0
%   .pct_status_2          fraction with status 2 (QP failure / max iter)
%   .pct_status_4          fraction with status 4 (QP infeasible)
%   .pct_status_other      everything else (most likely indicates a build/IO issue)
%   .all_converged         logical, true iff every step had status==0
%
% Useful in attribution plots (Stage B / C / H) and for ranking candidates.

    n = max(R.n_actual, 1);
    status = R.status_seq(1:R.n_actual);

    S.sqp_iter_mean = mean_(R.sqp_iter_seq);
    S.sqp_iter_p95  = pctile_(R.sqp_iter_seq, 95);
    S.sqp_iter_max  = max_(R.sqp_iter_seq);

    S.qp_iter_mean  = mean_(R.qp_iter_seq);
    S.qp_iter_p95   = pctile_(R.qp_iter_seq, 95);
    S.qp_iter_max   = max_(R.qp_iter_seq);

    S.pct_status_0     = 100 * sum(status == 0) / n;
    S.pct_status_2     = 100 * sum(status == 2) / n;
    S.pct_status_4     = 100 * sum(status == 4) / n;
    S.pct_status_other = 100 * sum(~ismember(status, [0 2 4])) / n;
    S.all_converged    = all(status == 0);
end

function m = mean_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = mean(x); end
end

function m = max_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = max(x); end
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
