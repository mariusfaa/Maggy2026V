function plot_failure_map(T, out_dir)
% PLOT_FAILURE_MAP  Grid of (variant, n, rep) cells colored by failure category.

    variants = ["full","reduced"];
    cat_levels = {'ok','no_equilibrium','compiler_killed','other_failure'};
    cat_colors = [ ...
        0.20 0.65 0.30;   % ok            green
        0.95 0.75 0.20;   % no_equilibrium yellow
        0.85 0.20 0.20;   % compiler_killed red
        0.50 0.50 0.50];  % other          grey

    f = figure('Position', [100 100 1000 460]);
    tlay = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    for vi = 1:numel(variants)
        v = variants(vi);
        Tv = T(string(T.variant) == v, :);
        if isempty(Tv); continue; end
        ns  = unique(Tv.magnet_n);
        rps = unique(Tv.repetition);
        M = zeros(numel(rps), numel(ns));   % cat index per cell
        for i = 1:numel(rps)
            for j = 1:numel(ns)
                row = Tv(Tv.magnet_n == ns(j) & Tv.repetition == rps(i), :);
                if isempty(row)
                    M(i, j) = NaN;
                else
                    c = char(row.failure_category(1));
                    M(i, j) = find(strcmp(cat_levels, c));
                end
            end
        end

        nexttile;
        imagesc(M, 'AlphaData', ~isnan(M));
        colormap(gca, cat_colors);
        caxis([1 numel(cat_levels)]);
        set(gca, 'XTick', 1:numel(ns), 'XTickLabel', string(ns), ...
                 'YTick', 1:numel(rps), 'YTickLabel', "rep " + string(rps), ...
                 'YDir', 'normal');
        xlabel('magnet n');
        title(char(v));
        axis image;
        box on;

        for i = 1:numel(rps)
            for j = 1:numel(ns)
                if isnan(M(i, j)); continue; end
                lab = cat_levels{M(i, j)};
                if strcmp(lab, 'ok'); lab = ''; end
                text(j, i, lab, 'HorizontalAlignment', 'center', ...
                     'VerticalAlignment', 'middle', 'FontSize', 8, ...
                     'Color', 'k', 'Interpreter', 'none');
            end
        end
    end

    title(tlay, 'Run status by variant, n, and repetition');

    % shared legend via invisible patches
    hold on;
    handles = gobjects(1, numel(cat_levels));
    for k = 1:numel(cat_levels)
        handles(k) = patch(NaN, NaN, cat_colors(k, :), 'EdgeColor', 'none', ...
            'DisplayName', cat_levels{k});
    end
    lgd = legend(handles, 'Orientation', 'horizontal', 'Box', 'off', ...
        'Interpreter', 'none');
    lgd.Layout.Tile = 'south';

    save_fig(f, out_dir, 'failure_map');
end
