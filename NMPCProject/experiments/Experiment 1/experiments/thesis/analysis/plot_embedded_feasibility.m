function fig = plot_embedded_feasibility(T, order)
% PLOT_EMBEDDED_FEASIBILITY  Stage D heatmap: t_p99 / Ts ratio over (N, Ts).
%
%   fig = plot_embedded_feasibility(T, order)
%
% The ratio t_p99_ms / (Ts*1000) is the dimensionless "embedded budget
% used" -- 1.0 means the solver exactly saturates the control period at
% the p99 level; < 0.8 means we have at least 20% headroom for timing
% jitter (the embedded-feasibility threshold). Tiles with no run are left
% white. Tiles where the cfg did not produce CONVERGED_STABLE are hatched.

    sub = T(strcmp(T.order, order), :);
    if isempty(sub)
        error('plot_embedded_feasibility:no_data', 'No rows for order=%s', order);
    end

    Ns  = sort(unique(sub.N));
    Tss = sort(unique(sub.Ts));
    Ts_ms = Tss * 1000;

    R = nan(numel(Ns), numel(Tss));   % ratio per cell
    S = false(numel(Ns), numel(Tss)); % stable flag per cell
    for r = 1:height(sub)
        iN = find(Ns  == sub.N(r),  1);
        iT = find(Tss == sub.Ts(r), 1);
        if isempty(iN) || isempty(iT); continue; end
        R(iN, iT) = sub.t_p99_over_Ts(r);
        S(iN, iT) = startsWith(sub.classification{r}, 'CONVERGED_STABLE');
    end

    fig = figure('Name', sprintf('Stage D embedded feasibility -- %s', order), ...
                 'Position', [50 50 1100 700]);
    ax = axes(fig);

    h = imagesc(ax, R);
    set(h, 'AlphaData', ~isnan(R));     % white for empty cells
    colormap(ax, custom_cmap_());       % green-yellow-red around 0.8
    cl = clim(ax);                       %#ok<NASGU>
    clim(ax, [0 2]);                    % cap at 200% of period
    cb = colorbar(ax);
    cb.Label.String = 't_{p99} / Ts';

    hold(ax,'on');
    % Mark not-stable cells with an X
    [iN_bad, iT_bad] = find(~S & ~isnan(R));
    for k = 1:numel(iN_bad)
        plot(ax, iT_bad(k), iN_bad(k), 'kx', 'MarkerSize', 12, 'LineWidth', 1.5);
    end

    % Tile labels (ratio)
    for i = 1:numel(Ns)
        for j = 1:numel(Tss)
            if isnan(R(i,j)); continue; end
            text(ax, j, i, sprintf('%.2f', R(i,j)), ...
                 'HorizontalAlignment','center', 'FontSize', 8);
        end
    end

    % Baseline marker: N=20, Ts=10 ms
    iN_b = find(Ns  == 20, 1);
    iT_b = find(Tss == 0.010, 1);
    if ~isempty(iN_b) && ~isempty(iT_b)
        rectangle(ax, 'Position', [iT_b-0.5, iN_b-0.5, 1, 1], ...
                  'EdgeColor', [0.10 0.30 0.80], 'LineWidth', 2.5, ...
                  'FaceColor','none');
    end

    ax.XTick      = 1:numel(Tss);
    ax.XTickLabel = arrayfun(@(x) sprintf('%g', x), Ts_ms, 'UniformOutput', false);
    ax.YTick      = 1:numel(Ns);
    ax.YTickLabel = arrayfun(@(x) sprintf('%d', x), Ns, 'UniformOutput', false);
    xlabel(ax, 'Ts [ms]');
    ylabel(ax, 'N (horizon)');
    title(ax, sprintf(['Stage D embedded feasibility -- %s   ' ...
                       '(< 0.8 = real-time feasible at p99; x = unstable)'], order), ...
          'Interpreter','none');
    axis(ax, 'image');
    box(ax, 'on');
end

function cm = custom_cmap_()
    % Green at 0..0.5, yellow at 0.8, orange at 1.0, red beyond.
    cm = [linspace(0.10, 1.00, 128)' linspace(0.60, 0.20, 128)' linspace(0.30, 0.10, 128)'];
end
