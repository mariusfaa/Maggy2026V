%% simBenchMpcCompare — Parse all MPC benchmark results and export CSV
%
% Loads all mpc_*.mat files from mpcresults/, extracts config and timing,
% computes tracking metrics, prints summary table, and exports CSV.

clear; clc;

mpc_dir  = 'mpcresults';
csv_file = 'mpcresults/mpc_benchmark.csv';

files = dir(fullfile(mpc_dir, 'mpc_*.mat'));
nFiles = numel(files);
assert(nFiles > 0, 'No mpc_*.mat files found in %s/', mpc_dir);
fprintf('Found %d MPC result files in %s/\n\n', nFiles, mpc_dir);

%% ================================================================
%% Parse all files
%% ================================================================

rows = cell(nFiles, 1);
valid = false(nFiles, 1);

for i = 1:nFiles
    filepath = fullfile(files(i).folder, files(i).name);
    [~, fname] = fileparts(files(i).name);

    try
        d = load(filepath);
    catch
        fprintf('  Skipping (load error): %s\n', fname);
        continue;
    end

    if ~isfield(d, 'config')
        fprintf('  Skipping (no config): %s\n', fname);
        continue;
    end

    cfg = d.config;
    r = struct();
    r.name = fname;

    % Config fields
    r.nlp_solver = cfg.nlp_solver;
    r.qp_solver  = cfg.qp_solver;
    r.N_horizon  = cfg.N_horizon;
    r.dt_mpc_ms  = cfg.dt_mpc * 1e3;
    r.Tf_ms      = cfg.dt_mpc * cfg.N_horizon * 1e3;
    r.stages     = cfg.ocp_stages;
    r.n_radial   = cfg.n_radial;
    r.diverged   = d.diverged;

    % Build time
    if isfield(d, 't_build')
        r.build_s = d.t_build;
    else
        r.build_s = NaN;
    end

    % Simulation length
    r.T_end = d.t(end);
    r.n_mpc_steps = numel(d.t_mpc);

    % Timing stats (us) — skip first 5 steps (JIT warmup)
    skip = min(5, numel(d.t_mpc) - 1);
    if numel(d.t_mpc) > skip
        tm = d.t_mpc(skip+1:end);
        tl = d.t_lin(skip+1:end);
        tq = d.t_qp(skip+1:end);
        tr = d.t_reg(skip+1:end);
        ts = d.t_sim(skip+1:end);
        tt = d.t_total(skip+1:end);
    else
        tm = d.t_mpc; tl = d.t_lin; tq = d.t_qp;
        tr = d.t_reg; ts = d.t_sim; tt = d.t_total;
    end

    r.mpc_mean_us    = mean(tm) * 1e6;
    r.mpc_median_us  = median(tm) * 1e6;
    r.mpc_max_us     = max(tm) * 1e6;
    r.lin_mean_us    = mean(tl) * 1e6;
    r.lin_median_us  = median(tl) * 1e6;
    r.qp_mean_us     = mean(tq) * 1e6;
    r.qp_median_us   = median(tq) * 1e6;
    r.reg_mean_us    = mean(tr) * 1e6;
    r.sim_mean_us    = mean(ts) * 1e6;
    r.sim_median_us  = median(ts) * 1e6;
    r.total_mean_us  = mean(tt) * 1e6;
    r.total_median_us = median(tt) * 1e6;
    r.total_max_us   = max(tt) * 1e6;

    % Real-time factor
    r.rt_factor = cfg.dt_mpc / mean(tt);

    % SQP iterations
    if isfield(d, 'sqp_iter')
        sq = d.sqp_iter(skip+1:end);
        r.sqp_iter_mean = mean(sq);
        r.sqp_iter_max  = max(sq);
    else
        r.sqp_iter_mean = NaN;
        r.sqp_iter_max  = NaN;
    end

    % Solver status summary
    if isfield(d, 'status')
        r.n_status_ok   = sum(d.status == 0 | d.status == 2);
        r.n_status_warn = sum(d.status ~= 0 & d.status ~= 2);
    else
        r.n_status_ok = NaN;
        r.n_status_warn = NaN;
    end

    % Tracking: final position/angle error
    if ~d.diverged && size(d.x, 2) > 0
        x_final = d.x(:, end);
        xEq = d.xEq;
        if numel(x_final) == 10
            r.pos_err_mm  = norm(x_final(1:3) - xEq(1:3)) * 1e3;
            r.ang_err_deg = norm(x_final(4:5) - xEq(4:5)) * 180/pi;
        else
            r.pos_err_mm  = norm(x_final(1:3) - xEq(1:3)) * 1e3;
            r.ang_err_deg = norm(x_final(4:6) - xEq(4:6)) * 180/pi;
        end
    else
        r.pos_err_mm  = NaN;
        r.ang_err_deg = NaN;
    end

    % Cost at end
    if isfield(d, 'cost') && ~d.diverged
        r.cost_final = d.cost(end);
        r.cost_mean  = mean(d.cost);
    else
        r.cost_final = NaN;
        r.cost_mean  = NaN;
    end

    % Timing breakdown percentages
    r.pct_lin = r.lin_mean_us / r.mpc_mean_us * 100;
    r.pct_qp  = r.qp_mean_us / r.mpc_mean_us * 100;
    r.pct_reg = r.reg_mean_us / r.mpc_mean_us * 100;

    rows{i} = r;
    valid(i) = true;
end

rows = rows(valid);
nR = numel(rows);
R = [rows{:}];
fprintf('Parsed %d / %d files.\n\n', nR, nFiles);

%% ================================================================
%% Build and display table
%% ================================================================

NLP        = {R.nlp_solver}';
QP         = {R.qp_solver}';
N          = [R.N_horizon]';
dt_ms      = [R.dt_mpc_ms]';
Tf_ms      = [R.Tf_ms]';
Stages     = [R.stages]';
Diverged   = [R.diverged]';
T_end      = [R.T_end]';
MPC_steps  = [R.n_mpc_steps]';
Build_s    = [R.build_s]';
MPC_med_us = [R.mpc_median_us]';
Lin_med_us = [R.lin_median_us]';
QP_med_us  = [R.qp_median_us]';
Sim_med_us = [R.sim_median_us]';
Tot_med_us = [R.total_median_us]';
Tot_max_us = [R.total_max_us]';
RT_factor  = [R.rt_factor]';
Pct_lin    = [R.pct_lin]';
Pct_qp     = [R.pct_qp]';
SQP_mean   = [R.sqp_iter_mean]';
SQP_max    = [R.sqp_iter_max]';
Warns      = [R.n_status_warn]';
PosErr_mm  = [R.pos_err_mm]';
AngErr_deg = [R.ang_err_deg]';
Cost_final = [R.cost_final]';

T = table(NLP, QP, N, dt_ms, Tf_ms, Stages, Diverged, T_end, MPC_steps, ...
          Build_s, MPC_med_us, Lin_med_us, QP_med_us, Sim_med_us, ...
          Tot_med_us, Tot_max_us, RT_factor, ...
          Pct_lin, Pct_qp, SQP_mean, SQP_max, Warns, ...
          PosErr_mm, AngErr_deg, Cost_final);

% Sort: non-diverged first, then by RT factor descending
[~, si] = sortrows([Diverged, -RT_factor]);
T = T(si, :);

fprintf('============================================================\n');
fprintf('  MPC BENCHMARK COMPARISON\n');
fprintf('============================================================\n\n');
disp(T);

% Highlight best real-time configs (non-diverged)
mask = ~Diverged;
if any(mask)
    subset = T(T.Diverged == false, :);
    [~, best_rt] = max(subset.RT_factor);
    fprintf('Best RT (non-diverged): N=%d dt=%.0fms -> RT=%.2fx (mpc=%.0f us, tot=%.0f us)\n', ...
        subset.N(best_rt), subset.dt_ms(best_rt), ...
        subset.RT_factor(best_rt), subset.MPC_med_us(best_rt), subset.Tot_med_us(best_rt));

    [~, best_err] = min(subset.PosErr_mm);
    fprintf('Best tracking (non-div): N=%d dt=%.0fms -> pos=%.4f mm, ang=%.4f deg\n', ...
        subset.N(best_err), subset.dt_ms(best_err), ...
        subset.PosErr_mm(best_err), subset.AngErr_deg(best_err));
end

%% ================================================================
%% Export CSV
%% ================================================================

writetable(T, csv_file);
fprintf('\nCSV exported to: %s\n', csv_file);
fprintf('Done.\n');
