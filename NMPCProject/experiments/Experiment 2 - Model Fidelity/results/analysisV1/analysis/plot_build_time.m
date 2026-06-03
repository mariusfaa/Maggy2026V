function plot_build_time(T, A, out_dir)
% PLOT_BUILD_TIME  Build time vs n (log-log) and stacked component breakdown.
    colors = struct('full', [0.10 0.40 0.85], 'reduced', [0.85 0.30 0.10]);

    % --- Figure 1: total build time vs n -----------------------------------
    f1 = figure('Position', [100 100 760 520]);
    hold on; grid on; box on;
    variants = ["full","reduced"];
    legends = {};
    for v = variants
        ok = T(string(T.variant) == v & double(T.completed) == 1, :);
        if isempty(ok); continue; end
        c = colors.(v);
        scatter(ok.magnet_n, ok.build_total_time_s, 36, c, 'filled', ...
            'MarkerFaceAlpha', 0.35, 'MarkerEdgeColor', 'none');
        Av = A(string(A.variant) == v, :);
        Av = Av(~isnan(Av.build_total_time_s_mean), :);
        Av = sortrows(Av, 'magnet_n');
        plot(Av.magnet_n, Av.build_total_time_s_mean, '-o', ...
            'Color', c, 'LineWidth', 2, 'MarkerFaceColor', c);
        legends{end+1} = char(v) + " runs";        %#ok<AGROW>
        legends{end+1} = char(v) + " mean";        %#ok<AGROW>
    end
    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('magnet discretization n');
    ylabel('total build time [s]');
    title('Build time vs n (log-log)');
    legend(legends, 'Location', 'northwest');
    save_fig(f1, out_dir, 'build_time');

    % --- Figure 2: stacked component breakdown -----------------------------
    f2 = figure('Position', [100 100 980 520]);
    components = {'graph_build_time_s', 'build_ocp_time_s', 'build_sim_time_s'};
    comp_labels = {'graph build', 'OCP build', 'SIM build'};
    tlay = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    for v = variants
        nexttile;
        Av = A(string(A.variant) == v, :);
        Av = sortrows(Av(~isnan(Av.build_total_time_s_mean), :), 'magnet_n');
        if isempty(Av)
            title(char(v) + " (no data)"); continue;
        end
        Y = zeros(height(Av), numel(components));
        for c = 1:numel(components)
            Y(:, c) = Av.([components{c} '_mean']);
        end
        bar(categorical(string(Av.magnet_n)), Y, 'stacked');
        grid on; box on;
        ylabel('time [s]');
        xlabel('magnet n');
        title(char(v) + " build breakdown");
        if v == variants(1)
            legend(comp_labels, 'Location', 'northwest');
        end
    end
    title(tlay, 'Build time component breakdown (mean over reps)');
    save_fig(f2, out_dir, 'build_time_components');
end
