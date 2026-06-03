function plot_code_size(T, A, out_dir)
% PLOT_CODE_SIZE  Per-run generated-code size vs n, full vs reduced.
%   Uses per_run_bytes (filtered by run_id) — NOT the CSV's cumulative bytes.
    colors = struct('full', [0.10 0.40 0.85], 'reduced', [0.85 0.30 0.10]);
    variants = ["full","reduced"];

    f = figure('Position', [100 100 820 560]);
    hold on; grid on; box on;
    legends = {};
    for v = variants
        ok = T(string(T.variant) == v & double(T.completed) == 1 ...
               & T.per_run_bytes > 0, :);
        if isempty(ok); continue; end
        c = colors.(v);
        scatter(ok.magnet_n, ok.per_run_bytes / 1e6, 40, c, 'filled', ...
            'MarkerFaceAlpha', 0.4, 'MarkerEdgeColor', 'none');
        Av = A(string(A.variant) == v, :);
        Av = sortrows(Av(~isnan(Av.per_run_bytes_mean) & Av.per_run_bytes_mean > 0, :), 'magnet_n');
        plot(Av.magnet_n, Av.per_run_bytes_mean / 1e6, '-o', ...
            'Color', c, 'LineWidth', 2, 'MarkerFaceColor', c);
        legends{end+1} = char(v) + " runs";        %#ok<AGROW>
        legends{end+1} = char(v) + " mean";        %#ok<AGROW>
    end
    set(gca, 'XScale', 'log', 'YScale', 'log');
    xlabel('magnet discretization n');
    ylabel('per-run generated code size [MB]');
    title('Generated C code size vs n');
    legend(legends, 'Location', 'northwest');

    note = {
        'QP solver (both variants):'
        '  PARTIAL\_CONDENSING\_HPIPM'
    };
    text(0.98, 0.02, note, 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
        'FontSize', 9, 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.7 0.7 0.7]);

    save_fig(f, out_dir, 'code_size');
end
