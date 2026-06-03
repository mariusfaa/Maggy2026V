function fig = plot_stability_region(T, order)
% PLOT_STABILITY_REGION  Stage D heatmap: outcome class over (N, Ts).
%
%   fig = plot_stability_region(T, order)
%
% X-axis: Ts (control period, ms). Y-axis: N (horizon length). Each tile
% is coloured by classification class. Tiles at (N, Ts) with no run
% (filtered out by Tf bounds in stageD) are left white. The Stage-A
% baseline at (N=20, Ts=10 ms) is marked with a blue square outline.

    sub = T(strcmp(T.order, order), :);
    if isempty(sub)
        error('plot_stability_region:no_data', 'No rows for order=%s', order);
    end

    Ns  = sort(unique(sub.N));
    Tss = sort(unique(sub.Ts));
    Ts_ms = Tss * 1000;

    % Build (N x Ts) class index
    class_names = { 'CONVERGED_STABLE', 'CONVERGED_STABLE_DIST_RECOVERED', ...
                    'CONVERGED_STABLE_DIST_FAILED', ...
                    'NLP_MAX_ITER', 'QP_INFEASIBLE', 'DIVERGED_PHYSICAL', ...
                    'NUMERIC_NAN', 'BUILD_FAIL' };
    class_idx = containers.Map(class_names, 1:numel(class_names));

    Z = nan(numel(Ns), numel(Tss));     % class index per cell
    P = nan(numel(Ns), numel(Tss));     % t_p99 in ms per cell
    for r = 1:height(sub)
        iN = find(Ns  == sub.N(r),  1);
        iT = find(Tss == sub.Ts(r), 1);
        if isempty(iN) || isempty(iT); continue; end
        cl = sub.classification{r};
        if class_idx.isKey(cl)
            Z(iN, iT) = class_idx(cl);
        end
        P(iN, iT) = sub.t_p99_ms(r);
    end

    fig = figure('Name', sprintf('Stage D stability region -- %s', order), ...
                 'Position', [50 50 1100 700]);
    ax = axes(fig);
    hold(ax,'on');

    % Draw each cell as a coloured rectangle
    for i = 1:numel(Ns)
        for j = 1:numel(Tss)
            if isnan(Z(i,j)); continue; end
            c = class_color_(class_names{Z(i,j)});
            rectangle(ax, 'Position', [j-0.5, i-0.5, 1, 1], ...
                      'FaceColor', c, 'EdgeColor', [0.4 0.4 0.4]);
            if ~isnan(P(i,j))
                text(ax, j, i, sprintf('%.1f', P(i,j)), ...
                     'HorizontalAlignment','center', 'FontSize', 8, ...
                     'Color', text_color_(c));
            end
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
    title(ax, sprintf('Stage D stability region -- %s   (tile value = t_{p99} ms)', order), ...
          'Interpreter','none');
    axis(ax, [0.5 numel(Tss)+0.5 0.5 numel(Ns)+0.5]);
    box(ax, 'on');

    % Manual legend
    leg_classes = {'CONVERGED_STABLE', 'QP_INFEASIBLE', 'DIVERGED_PHYSICAL', ...
                   'NUMERIC_NAN', 'BUILD_FAIL'};
    leg_handles = gobjects(1, numel(leg_classes));
    for k = 1:numel(leg_classes)
        leg_handles(k) = patch(ax, NaN, NaN, class_color_(leg_classes{k}), ...
                               'EdgeColor',[0.4 0.4 0.4], 'DisplayName', leg_classes{k});
    end
    legend(leg_handles, 'Interpreter','none', 'Location','northeastoutside');
end

function c = class_color_(name)
    switch true
        case isempty(name);                       c = [1 1 1];
        case startsWith(name,'CONVERGED_STABLE'); c = [0.20 0.60 0.30];
        case strcmp(name,'DIVERGED_PHYSICAL');    c = [0.85 0.20 0.20];
        case strcmp(name,'QP_INFEASIBLE');        c = [0.55 0.10 0.10];
        case strcmp(name,'NLP_MAX_ITER');         c = [0.95 0.50 0.10];
        case strcmp(name,'NUMERIC_NAN');          c = [0.05 0.05 0.05];
        case strcmp(name,'BUILD_FAIL');           c = [0.5 0.5 0.5];
        otherwise;                                c = [0.7 0.7 0.7];
    end
end

function tc = text_color_(bg)
    if mean(bg) < 0.45; tc = [1 1 1]; else; tc = [0 0 0]; end
end
