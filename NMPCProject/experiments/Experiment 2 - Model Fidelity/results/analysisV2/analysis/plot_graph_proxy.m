function plot_graph_proxy(T, A, out_dir)
% PLOT_GRAPH_PROXY  CasADi symbolic-graph size proxy: dynamics-only .c line count.
%   The dynamics-only count excludes acados QP / mex / sim-solver glue, so it
%   isolates growth coming from the symbolic model itself (n-dependent
%   trapezoidal field integral).
    colors = struct('full', [0.10 0.40 0.85], 'reduced', [0.85 0.30 0.10]);
    variants = ["full","reduced"];

    f = figure('Position', [100 100 820 560]);
    hold on; grid on; box on;
    legends = {};
    for v = variants
        ok = T(string(T.variant) == v & double(T.completed) == 1 ...
               & T.per_run_dyn_c_lines > 0, :);
        if isempty(ok); continue; end
        c = colors.(v);
        scatter(ok.magnet_n, ok.per_run_dyn_c_lines, 40, c, 'filled', ...
            'MarkerFaceAlpha', 0.4, 'MarkerEdgeColor', 'none');
        Av = A(string(A.variant) == v, :);
        Av = sortrows(Av(~isnan(Av.per_run_dyn_c_lines_mean) ...
                         & Av.per_run_dyn_c_lines_mean > 0, :), 'magnet_n');
        plot(Av.magnet_n, Av.per_run_dyn_c_lines_mean, '-o', ...
            'Color', c, 'LineWidth', 2, 'MarkerFaceColor', c);
        legends{end+1} = char(v) + " runs"; %#ok<AGROW>
        legends{end+1} = char(v) + " mean"; %#ok<AGROW>
    end
    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('magnet discretization n');
    ylabel('dynamics-related .c lines (proxy for symbolic graph size)');
    title('CasADi symbolic graph size proxy vs n');
    legend(legends, 'Location', 'northwest');
    save_fig(f, out_dir, 'graph_proxy');
end
