function analyze_sweep()
% ANALYZE_SWEEP  Generate thesis plots and aggregate CSV from the n-sweep results.
%
%   Run from MATLAB (any cwd):
%       run('/home/mariujf/n_sweep/.../results_magnet_n_sweep/analysis/analyze_sweep.m')
%   or:
%       cd /.../results_magnet_n_sweep/analysis
%       analyze_sweep
%
%   Outputs (in this directory):
%       aggregated.csv
%       build_time.{png,pdf}, build_time_components.{png,pdf}
%       solve_time.{png,pdf}, solve_time_max.{png,pdf}
%       code_size.{png,pdf}
%       graph_proxy.{png,pdf}
%       failure_map.{png,pdf}

    here = fileparts(mfilename('fullpath'));
    addpath(here);
    results_root = fullfile(here, '..');
    csv_path     = fullfile(results_root, 'magnet_n_sweep_summary.csv');
    code_root    = fullfile(results_root, 'generated_code');
    out_dir      = here;

    fprintf('[analyze_sweep] reading %s\n', csv_path);
    T = readtable(csv_path, 'TextType', 'string');
    fprintf('[analyze_sweep] %d rows, %d variants, n range [%d, %d]\n', ...
        height(T), numel(unique(T.variant)), min(T.magnet_n), max(T.magnet_n));

    T = classify_failures(T);

    fprintf('[analyze_sweep] measuring per-run generated code...\n');
    t0 = tic;
    T = measure_generated_code(T, code_root);
    fprintf('[analyze_sweep]   done in %.1f s\n', toc(t0));

    value_cols = { ...
        'graph_build_time_s', 'build_ocp_time_s', 'build_sim_time_s', ...
        'build_total_time_s', 'total_wall_time_s', ...
        'solve_wall_mean_ms', 'solve_wall_median_ms', ...
        'solve_wall_p95_ms', 'solve_wall_max_ms', ...
        'generated_code_bytes', 'per_run_bytes', ...
        'per_run_c_lines', 'per_run_dyn_c_lines', ...
        'sqp_iter_mean', 'sqp_iter_max'};
    A = aggregate_by_n(T, value_cols);

    agg_csv = fullfile(out_dir, 'aggregated.csv');
    writetable(A, agg_csv);
    fprintf('[analyze_sweep] wrote %s\n', agg_csv);

    aug_csv = fullfile(out_dir, 'per_run_with_code_size.csv');
    writetable(T, aug_csv);
    fprintf('[analyze_sweep] wrote %s\n', aug_csv);

    fprintf('[analyze_sweep] plotting...\n');
    plot_build_time(T, A, out_dir);
    plot_solve_time(T, A, out_dir);
    plot_code_size(T, A, out_dir);
    plot_graph_proxy(T, A, out_dir);
    plot_failure_map(T, out_dir);

    print_summary(T, A);
    fprintf('[analyze_sweep] done.\n');
end

function print_summary(T, A)
    fprintf('\n=== Summary ===\n');
    n_ok = sum(double(T.completed) == 1);
    fprintf('Completed runs:  %d / %d\n', n_ok, height(T));
    cats = T.failure_category;
    levels = categories(cats);
    for i = 1:numel(levels)
        if strcmp(levels{i}, 'ok'); continue; end
        c = sum(cats == levels{i});
        if c > 0
            fprintf('  %-18s %d\n', levels{i}, c);
        end
    end

    fprintf('\nPer-variant size at n=6 (mean of per_run_bytes):\n');
    for v = ["full","reduced"]
        row = A(string(A.variant) == v & A.magnet_n == 6, :);
        if ~isempty(row)
            fprintf('  %-8s %8.1f MB\n', v, row.per_run_bytes_mean / 1e6);
        end
    end
    fprintf('\nLargest successful per_run_bytes per variant:\n');
    for v = ["full","reduced"]
        Av = A(string(A.variant) == v & ~isnan(A.per_run_bytes_mean), :);
        if isempty(Av); continue; end
        [val, k] = max(Av.per_run_bytes_mean);
        fprintf('  %-8s n=%-4d %8.1f MB\n', v, Av.magnet_n(k), val/1e6);
    end
    fprintf('\n');
end
