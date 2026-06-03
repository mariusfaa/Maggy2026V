function fig = plot_main_effects(T, order, varargin)
% PLOT_MAIN_EFFECTS  Small-multiples bar chart for Stage B.
%
%   fig = plot_main_effects(T, order)
%
% One panel per factor. Each panel shows bars for every level, grouped by
% IC variant. Bar height is the acados p99 solve time in ms (the primary
% deployment-relevant timing metric). Bars are coloured by outcome class:
%
%   CONVERGED_STABLE          green
%   CONVERGED_STABLE_DIST_*   green/cyan
%   DIVERGED_PHYSICAL         red
%   QP_INFEASIBLE             dark red
%   NLP_MAX_ITER              orange
%   NUMERIC_NAN               black
%   BUILD_FAIL                grey
%
% Bars marked with a black-edged star above their top if
% embedded_feasible_p99 is FALSE (so the user can see at a glance which
% configurations would miss the real-time deadline at the baseline Ts).
%
% Inputs
%   T      table from stageB_solver_factors().report.table
%   order  'full12' or 'reduced10'

    p = inputParser();
    p.addRequired('T', @istable);
    p.addRequired('order', @ischar);
    p.parse(T, order, varargin{:});

    sub = T(strcmp(T.order, order), :);
    if isempty(sub)
        error('plot_main_effects:no_data', 'No rows for order=%s', order);
    end

    factors  = unique(sub.factor, 'stable');
    n_facets = numel(factors);

    fig = figure('Name', sprintf('Stage B main effects -- %s', order), ...
                 'Position', [50 50 1400 900]);

    cols = ceil(sqrt(n_facets));
    rows = ceil(n_facets / cols);

    for fi = 1:n_facets
        f = factors{fi};
        sub_f = sub(strcmp(sub.factor, f), :);

        levels = unique(sub_f.level, 'stable');
        ics    = unique(sub_f.ic_label, 'stable');

        Y = nan(numel(levels), numel(ics));
        C = repmat({''}, numel(levels), numel(ics));
        E = false(numel(levels), numel(ics));
        for li = 1:numel(levels)
            for ii = 1:numel(ics)
                m = strcmp(sub_f.level, levels{li}) & strcmp(sub_f.ic_label, ics{ii});
                if any(m)
                    Y(li,ii) = sub_f.t_p99_ms(find(m,1,'first'));
                    C{li,ii} = sub_f.classification{find(m,1,'first')};
                    E(li,ii) = ~sub_f.embedded_feasible_p99(find(m,1,'first'));
                end
            end
        end

        ax = subplot(rows, cols, fi);
        hold(ax, 'on'); grid(ax, 'on');
        b = bar(ax, Y, 'grouped');

        % Color by class for each (level, ic) group.
        for ii = 1:numel(b)
            face_colors = zeros(numel(levels), 3);
            for li = 1:numel(levels)
                face_colors(li, :) = class_color(C{li, ii});
            end
            b(ii).FaceColor  = 'flat';
            b(ii).CData      = face_colors;
            b(ii).EdgeColor  = [0.2 0.2 0.2];
        end

        % Stars for not-real-time-feasible-at-p99 entries.
        x_centers = bar_group_centers(b);
        for ii = 1:numel(b)
            for li = 1:numel(levels)
                if E(li, ii) && ~isnan(Y(li, ii))
                    plot(ax, x_centers(li, ii), Y(li, ii) * 1.05, ...
                         'k*', 'MarkerSize', 6);
                end
            end
        end

        ax.XTick      = 1:numel(levels);
        ax.XTickLabel = levels;
        ax.XTickLabelRotation = 30;
        ylabel(ax, 't_{p99} [ms]');
        title(ax, f, 'Interpreter', 'none');
        if fi == 1
            legend(ax, ics, 'Interpreter','none', 'Location','best');
        end
    end

    sgtitle(sprintf('Stage B main effects -- %s   (bar color = outcome class; * = embedded-infeasible at p99)', order), ...
            'Interpreter','none');
end

% ---------------------------------------------------------------------- %

function c = class_color(class_name)
    switch true
        case isempty(class_name);                       c = [0.7 0.7 0.7];
        case startsWith(class_name,'CONVERGED_STABLE'); c = [0.20 0.60 0.30];
        case strcmp(class_name,'CONVERGED_DEGRADED');   c = [0.95 0.75 0.20];
        case strcmp(class_name,'DIVERGED_PHYSICAL');    c = [0.85 0.20 0.20];
        case strcmp(class_name,'QP_INFEASIBLE');        c = [0.55 0.10 0.10];
        case strcmp(class_name,'NLP_MAX_ITER');         c = [0.95 0.50 0.10];
        case strcmp(class_name,'NUMERIC_NAN');          c = [0.05 0.05 0.05];
        case strcmp(class_name,'BUILD_FAIL');           c = [0.5 0.5 0.5];
        case strcmp(class_name,'TIMEOUT_PER_STEP');     c = [0.85 0.45 0.85];
        otherwise;                                       c = [0.7 0.7 0.7];
    end
end

function xc = bar_group_centers(b)
% Return the (n_groups x n_bars) x-center matrix for a 'grouped' bar.
    n_groups = numel(b(1).XData);
    n_bars   = numel(b);
    xc = zeros(n_groups, n_bars);
    for ii = 1:n_bars
        xc(:, ii) = b(ii).XEndPoints(:);
    end
end
