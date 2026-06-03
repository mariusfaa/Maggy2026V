function [class, reason] = classify_outcome(R, M, cfg)
% CLASSIFY_OUTCOME  Map a closed-loop run record to a structured outcome class.
%
%   [class, reason] = classify_outcome(R, M, cfg)
%
% Order of evaluation (FIRST MATCH WINS):
%   1. NUMERIC_NAN          any NaN/Inf in state, control, or timing
%   2. QP_INFEASIBLE        any per-step status in {2, 4}
%   3. NLP_MAX_ITER         SQP only: >= 5% of steps hit cfg.nlp_solver_max_iter
%   4. DIVERGED_PHYSICAL    R.diverged true (z/angle bounds exceeded)
%   5. CONVERGED_STABLE     everything else
%
% (BUILD_FAIL is set by the caller around build_ocp / build_plant and never
%  reaches this function.)
%
% The class is a STABILITY decision -- it answers the user's "successful"
% definition: the magnet remains levitated, the z-axis stays bounded near
% equilibrium, the system does not leave the platform, and the solver
% converges. Tracking quality (final error magnitude, settling time, peak
% deviation, integrated cost) is reported separately in the summary fields
% and is what stage filtering uses to RANK stable configurations.
%
% This deviates from the original plan in two ways:
%   - TIMEOUT_PER_STEP is dropped; real-time feasibility is reported via
%     timing_stats.embedded_feasible_{p99,max}.
%   - CONVERGED_DEGRADED is dropped; "stable but with X% final error" is
%     captured by the existing tracking metrics. Combining tracking quality
%     with stability into a single class makes filtering harder and would
%     mis-classify the full-12 baseline (uncontrollable yaw causes a
%     persistent non-zero final error even though the closed loop is
%     manifestly stable).
%
% Disturbance suffix
%   If cfg.disturbance_schedule is non-empty AND the run did NOT diverge,
%   the class is appended with "_DIST_RECOVERED" if the post-disturbance
%   error decays to <= 5% of the initial error within 2.0 s, else
%   "_DIST_FAILED".
%
% Inputs
%   R    struct from run_closed_loop
%   M    struct from build_model (uses M.xEq_ctrl)
%   cfg  validated cfg (uses .nlp_solver_type, .nlp_solver_max_iter,
%        .N, .Tf, .disturbance_schedule)
%
% Outputs
%   class   char, one of the names above
%   reason  short human-readable string citing the trigger

    n = R.n_actual;

    % --- 1. Numeric failure --------------------------------------------
    has_nan = any(~isfinite(R.X_plant(:))) ...
           || any(~isfinite(R.U(:))) ...
           || any(~isfinite(R.timing.tot)) ...
           || any(~isfinite(R.timing.wall));
    if has_nan
        class  = 'NUMERIC_NAN';
        first_nan_step = find_first_nan(R);
        reason = sprintf('NaN/Inf detected at or before step %d (n_actual=%d)', ...
                          first_nan_step, n);
        return
    end

    % --- 2. QP infeasibility ------------------------------------------
    bad_status = ismember(R.status_seq, [2 4]);
    if any(bad_status)
        first = find(bad_status, 1, 'first');
        class  = 'QP_INFEASIBLE';
        reason = sprintf('status=%d at step %d/%d (sqp_iter_mean=%.1f)', ...
                          R.status_seq(first), first, n, nanmean_(R.sqp_iter_seq));
        return
    end

    % --- 3. NLP max-iter (SQP only; RTI is forced to 1 by validate_cfg) -
    if strcmp(cfg.nlp_solver_type, 'SQP') && cfg.nlp_solver_max_iter > 1
        capped = R.sqp_iter_seq >= cfg.nlp_solver_max_iter;
        frac   = sum(capped) / max(n, 1);
        if frac >= 0.05
            class  = 'NLP_MAX_ITER';
            reason = sprintf('SQP hit max_iter=%d on %.1f%% of steps', ...
                              cfg.nlp_solver_max_iter, 100 * frac);
            return
        end
    end

    % --- 4. Physical divergence ---------------------------------------
    if R.diverged
        class  = 'DIVERGED_PHYSICAL';
        reason = sprintf('plant state exceeded physical bounds at step %d/%d', n, cfg.sim_steps);
        return
    end

    % (Real-time feasibility and tracking quality are reported orthogonally
    % via the summary fields and do not influence the stability class.)

    % --- 5. Default: stable -------------------------------------------
    base   = 'CONVERGED_STABLE';
    reason = sprintf('all %d steps converged within stability bounds', n);
    class  = with_dist_suffix(base, R, M, cfg);
end

% ---------------------------------------------------------------------- %

function class = with_dist_suffix(base, R, M, cfg)
    if isempty(cfg.disturbance_schedule)
        class = base;
        return
    end
    k_dist = R.disturbance_step;
    if isnan(k_dist) || k_dist >= R.n_actual
        class = [base '_DIST_FAILED'];
        return
    end
    % Error norm time series on controller side.
    err = vecnorm(R.X_ctrl - M.xEq_ctrl);
    Ts  = R.dt;
    recovery_window = round(2.0 / Ts);   % 2 s
    init_err = err(1);
    if init_err <= 0
        class = [base '_DIST_FAILED']; return
    end
    threshold = 0.05 * init_err;
    end_idx = min(k_dist + recovery_window, numel(err));
    window = err((k_dist+1):end_idx);
    if any(window <= threshold)
        class = [base '_DIST_RECOVERED'];
    else
        class = [base '_DIST_FAILED'];
    end
end

function step = find_first_nan(R)
    for j = 1:R.n_actual
        if any(~isfinite(R.X_plant(:, j+1))) ...
                || any(~isfinite(R.U(:, j))) ...
                || ~isfinite(R.timing.tot(j))
            step = j; return
        end
    end
    step = R.n_actual;
end

function m = nanmean_(x)
    x = x(~isnan(x));
    if isempty(x); m = NaN; else; m = mean(x); end
end
