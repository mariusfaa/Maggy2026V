function fig = plot_integrator_pareto(T, order, varargin)
% PLOT_INTEGRATOR_PARETO  Stage C accuracy-vs-speed Pareto, per integrator.
%
%   fig = plot_integrator_pareto(T, order)
%
% One panel per integrator_type. Each point is a (stages, steps) cfg from
% Stage C. X = p99 solve time in ms (acados time_tot, the deployment-
% relevant metric). Y = integrated cost J. Colour = outcome class:
%
%   CONVERGED_STABLE         green
%   NUMERIC_NAN              black
%   QP_INFEASIBLE            dark red
%   DIVERGED_PHYSICAL        red
%   NLP_MAX_ITER             orange
%   BUILD_FAIL               grey
%
% Point labels are "s<n_stages>,t<n_steps>". Stage-A baseline (s=4, t=10)
% is ringed in blue. Points with embedded_feasible_p99 == false get a black
% star above them (so the reader can see at a glance which configurations
% would miss the real-time deadline at the baseline Ts).

    p = inputParser();
    p.addRequired('T', @istable);
    p.addRequired('order', @ischar);
    p.parse(T, order, varargin{:});

    sub = T(strcmp(T.order, order), :);
    if isempty(sub)
        error('plot_integrator_pareto:no_data', 'No rows for order=%s', order);
    end

    types = unique(sub.integrator_type, 'stable');
    n_panels = numel(types);

    fig = figure('Name', sprintf('Stage C Pareto -- %s', order), ...
                 'Position', [50 50 1400 500]);

    for ti = 1:n_panels
        t = types{ti};
        st = sub(strcmp(sub.integrator_type, t), :);

        ax = subplot(1, n_panels, ti);
        hold(ax, 'on'); grid(ax, 'on');

        % Per-class scatter
        classes = unique(st.classification);
        for ci = 1:numel(classes)
            cl = classes{ci};
            mask = strcmp(st.classification, cl);
            x = st.t_p99_ms(mask);
            y = st.J_integrated(mask);
            scatter(ax, x, y, 60, ...
                'MarkerFaceColor', class_color(cl), ...
                'MarkerEdgeColor', [0.2 0.2 0.2], ...
                'DisplayName',     cl);
        end

        % Point labels
        for r = 1:height(st)
            if isnan(st.t_p99_ms(r)) || isnan(st.J_integrated(r)); continue; end
            text(ax, st.t_p99_ms(r), st.J_integrated(r), ...
                sprintf(' s%d,t%d', st.n_stages(r), st.n_steps(r)), ...
                'FontSize', 8, 'VerticalAlignment','bottom');
        end

        % Highlight Stage-A baseline (s=4, t=10)
        bl = strcmp(st.integrator_type, t) & st.n_stages == 4 & st.n_steps == 10;
        if any(bl)
            scatter(ax, st.t_p99_ms(bl), st.J_integrated(bl), 140, ...
                'MarkerEdgeColor', [0.10 0.30 0.80], 'LineWidth', 2.0, ...
                'MarkerFaceColor','none', 'HandleVisibility','off');
        end

        % Embedded-feasibility stars
        bad = ~st.embedded_feasible_p99;
        if any(bad)
            x = st.t_p99_ms(bad);
            y = st.J_integrated(bad);
            for k = 1:numel(x)
                if isnan(x(k)) || isnan(y(k)); continue; end
                plot(ax, x(k), y(k) * 1.05, 'k*', 'MarkerSize', 6, ...
                     'HandleVisibility','off');
            end
        end

        xlabel(ax, 't_{p99} [ms]');
        ylabel(ax, 'J_{integrated}');
        title(ax, t, 'Interpreter', 'none');
        if ti == 1
            legend(ax, 'Location', 'best', 'Interpreter','none');
        end
    end

    sgtitle(sprintf(['Stage C integrator-depth Pareto -- %s   ' ...
                     '(blue ring = baseline s4,t10; * = embedded-infeasible at p99)'], order), ...
            'Interpreter', 'none');
end

% ---------------------------------------------------------------------- %

function c = class_color(class_name)
    switch true
        case isempty(class_name);                       c = [0.7 0.7 0.7];
        case startsWith(class_name,'CONVERGED_STABLE'); c = [0.20 0.60 0.30];
        case strcmp(class_name,'DIVERGED_PHYSICAL');    c = [0.85 0.20 0.20];
        case strcmp(class_name,'QP_INFEASIBLE');        c = [0.55 0.10 0.10];
        case strcmp(class_name,'NLP_MAX_ITER');         c = [0.95 0.50 0.10];
        case strcmp(class_name,'NUMERIC_NAN');          c = [0.05 0.05 0.05];
        case strcmp(class_name,'BUILD_FAIL');           c = [0.5 0.5 0.5];
        otherwise;                                       c = [0.7 0.7 0.7];
    end
end
