function fig = plot_final_pareto(R)
% PLOT_FINAL_PARETO  Stage H -- final 3D Pareto across all surviving
% candidates from Stage F (the panel) + the per-order tuned config from
% Stage E.
%
%   fig = plot_final_pareto(R)
%
% Axes: (t_p99_ms [log], J_integrated [log], peak_pos_mm).
% Marker: circle = full12, square = reduced10. Colour by panel candidate.
% Marker size = 200/sqrt(std_tp99) (so tighter timing = larger marker).
% Two 2D projection subplots flank the 3D view.

    if isempty(R.F) || ~isfield(R.F,'table')
        error('plot_final_pareto:no_data','Stage F report missing');
    end
    T = R.F.table;
    % Only baseline IC for the 3D Pareto -- comparable across candidates
    T = T(strcmp(T.ic_label,'ic_baseline'), :);
    % Drop unstable
    T = T(startsWith(T.classification,'CONVERGED_STABLE'), :);

    fig = figure('Name','Stage H final Pareto', 'Position',[50 50 1500 600]);

    panels = unique(T.panel,'stable');
    cmap = lines(numel(panels));

    % --- 3D scatter ----------------------------------------------------
    ax3 = subplot(1,3,1);
    hold(ax3,'on'); grid(ax3,'on'); view(ax3, 45, 30);
    for pi = 1:numel(panels)
        for o = {'full12','reduced10'}
            m = strcmp(T.panel, panels{pi}) & strcmp(T.order, o{1});
            if ~any(m); continue; end
            marker = ternary_(strcmp(o{1},'full12'), 'o', 's');
            scatter3(ax3, T.t_p99_ms(m), T.J_integrated(m), T.peak_pos_mm(m), ...
                80, cmap(pi,:), 'filled', 'Marker', marker, ...
                'MarkerEdgeColor',[0.2 0.2 0.2], 'DisplayName', sprintf('%s/%s', panels{pi}, o{1}));
        end
    end
    set(ax3, 'XScale','log', 'YScale','log');
    xlabel(ax3, 't_{p99} [ms]'); ylabel(ax3, 'J_{integrated}'); zlabel(ax3, 'peak |pos| [mm]');
    title(ax3, '3D Pareto');

    % --- 2D: (t_p99, J) ------------------------------------------------
    ax1 = subplot(1,3,2);
    hold(ax1,'on'); grid(ax1,'on');
    for pi = 1:numel(panels)
        for o = {'full12','reduced10'}
            m = strcmp(T.panel, panels{pi}) & strcmp(T.order, o{1});
            if ~any(m); continue; end
            marker = ternary_(strcmp(o{1},'full12'),'o','s');
            scatter(ax1, T.t_p99_ms(m), T.J_integrated(m), 80, ...
                cmap(pi,:), 'filled', 'Marker', marker, ...
                'MarkerEdgeColor',[0.2 0.2 0.2]);
            text(ax1, T.t_p99_ms(m), T.J_integrated(m), ...
                sprintf(' %s/%s', panels{pi}, o{1}(1:3)), 'FontSize',8);
        end
    end
    set(ax1,'XScale','log','YScale','log');
    xlabel(ax1,'t_{p99} [ms]'); ylabel(ax1,'J_{integrated}');
    title(ax1,'Speed vs cost');

    % --- 2D: (t_p99, peak_pos) ----------------------------------------
    ax2 = subplot(1,3,3);
    hold(ax2,'on'); grid(ax2,'on');
    for pi = 1:numel(panels)
        for o = {'full12','reduced10'}
            m = strcmp(T.panel, panels{pi}) & strcmp(T.order, o{1});
            if ~any(m); continue; end
            marker = ternary_(strcmp(o{1},'full12'),'o','s');
            scatter(ax2, T.t_p99_ms(m), T.peak_pos_mm(m), 80, ...
                cmap(pi,:), 'filled', 'Marker', marker, ...
                'MarkerEdgeColor',[0.2 0.2 0.2]);
        end
    end
    set(ax2,'XScale','log');
    xlabel(ax2,'t_{p99} [ms]'); ylabel(ax2,'peak |pos| [mm]');
    title(ax2,'Speed vs peak deviation');

    legend(ax3, 'Interpreter','none', 'Location','northeastoutside');
    sgtitle('Stage H final Pareto  (o = full12, square = reduced10)');
end

function v = ternary_(cond, a, b)
    if cond; v = a; else; v = b; end
end
