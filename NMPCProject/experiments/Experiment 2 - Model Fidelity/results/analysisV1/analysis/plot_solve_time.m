function plot_solve_time(T, A, out_dir)
% PLOT_SOLVE_TIME  Mean wall solve time with p95 band, plus max solve time.
    colors = struct('full', [0.10 0.40 0.85], 'reduced', [0.85 0.30 0.10]);
    variants = ["full","reduced"];
    dt_ms = 5;  % NMPC sample time -> real-time threshold reference

    % --- Figure 1: mean with p95 band --------------------------------------
    f1 = figure('Position', [100 100 760 520]);
    hold on; grid on; box on;
    legends = {};
    for v = variants
        Av = A(string(A.variant) == v, :);
        Av = sortrows(Av(~isnan(Av.solve_wall_mean_ms_mean), :), 'magnet_n');
        if isempty(Av); continue; end
        c = colors.(v);
        x = Av.magnet_n;
        lo = Av.solve_wall_mean_ms_mean;
        hi = Av.solve_wall_p95_ms_mean;
        fill([x; flipud(x)], [lo; flipud(hi)], c, ...
            'FaceAlpha', 0.18, 'EdgeColor', 'none');
        plot(x, lo, '-o', 'Color', c, 'LineWidth', 2, 'MarkerFaceColor', c);
        legends{end+1} = char(v) + " mean–p95 band"; %#ok<AGROW>
        legends{end+1} = char(v) + " mean";           %#ok<AGROW>
    end
    yline(dt_ms, '--k', sprintf('NMPC dt = %g ms (real-time threshold)', dt_ms), ...
        'LabelHorizontalAlignment', 'left');
    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('magnet discretization n');
    ylabel('wall-clock solve time [ms]');
    title('Per-step OCP solve time vs n (mean + p95)');
    legend(legends, 'Location', 'northwest');
    save_fig(f1, out_dir, 'solve_time');

    % --- Figure 2: tail behavior (max) -------------------------------------
    f2 = figure('Position', [100 100 760 520]);
    hold on; grid on; box on;
    legends = {};
    for v = variants
        ok = T(string(T.variant) == v & double(T.completed) == 1, :);
        if isempty(ok); continue; end
        c = colors.(v);
        scatter(ok.magnet_n, ok.solve_wall_max_ms, 36, c, 'filled', ...
            'MarkerFaceAlpha', 0.4, 'MarkerEdgeColor', 'none');
        Av = A(string(A.variant) == v, :);
        Av = sortrows(Av(~isnan(Av.solve_wall_max_ms_mean), :), 'magnet_n');
        plot(Av.magnet_n, Av.solve_wall_max_ms_mean, '-o', ...
            'Color', c, 'LineWidth', 2, 'MarkerFaceColor', c);
        legends{end+1} = char(v) + " per-run max"; %#ok<AGROW>
        legends{end+1} = char(v) + " mean of max"; %#ok<AGROW>
    end
    yline(dt_ms, '--k', sprintf('NMPC dt = %g ms', dt_ms), ...
        'LabelHorizontalAlignment', 'left');
    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('magnet discretization n');
    ylabel('worst-case solve time [ms]');
    title('Worst-case per-step solve time vs n');
    legend(legends, 'Location', 'northwest');
    save_fig(f2, out_dir, 'solve_time_max');
end
