function fig = plot_trajectory_overlay(framework_R, baseline_data, varargin)
% PLOT_TRAJECTORY_OVERLAY  Overlay framework vs baseline closed-loop
% trajectories with a residual panel, for Stage A validation.
%
%   fig = plot_trajectory_overlay(framework_R, baseline_data)
%   fig = plot_trajectory_overlay(framework_R, baseline_data, ...
%             'Title', 'A1 full12 baseline', ...
%             'SavePath', '/path/to/figure')
%
% Inputs
%   framework_R    struct from run_closed_loop (uses .X_plant, .U,
%                  .status_seq, .timing.tot, .dt, .n_actual)
%   baseline_data  struct from the working simulator's saved .mat. Required
%                  fields (matching nmpc_results.mat layout):
%                    .x          12 x (n+1) or 10 x (n+1) plant trajectory
%                    .u          4 x n     applied inputs
%                    .dt         scalar Ts
%                  Optional:
%                    .status, .sqp_iter, .solve_time
%
% Output
%   fig            figure handle
%
% The framework is always 12-state on the plant side; the baseline may be
% 10-state (reduced) or 12-state (full). The plot projects the framework's
% 12-state trajectory to the baseline's order before overlaying so the
% comparison is apples-to-apples.

    p = inputParser();
    p.addRequired('framework_R',   @isstruct);
    p.addRequired('baseline_data', @isstruct);
    p.addParameter('Title',    'Stage-A trajectory overlay', @ischar);
    p.addParameter('SavePath', '',                            @ischar);
    p.parse(framework_R, baseline_data, varargin{:});
    ttl  = p.Results.Title;
    sp   = p.Results.SavePath;

    Xf = framework_R.X_plant;
    Uf = framework_R.U;
    dtf = framework_R.dt;
    nf  = framework_R.n_actual;

    Xb = baseline_data.x;
    Ub = baseline_data.u;
    dtb = baseline_data.dt;

    % --- Project framework to baseline's state dimension ---------------
    nxb = size(Xb, 1);
    if nxb == 10
        % Drop yaw (row 6) and yaw rate (row 12) from the 12-state framework.
        Xf_proj = Xf([1 2 3 4 5 7 8 9 10 11], :);
        state_names = {'x','y','z','roll','pitch','vx','vy','vz','wx','wy'};
        units = {'mm','mm','mm','deg','deg','mm/s','mm/s','mm/s','deg/s','deg/s'};
        scales = [1e3 1e3 1e3 180/pi 180/pi 1e3 1e3 1e3 180/pi 180/pi];
    elseif nxb == 12
        Xf_proj = Xf;
        state_names = {'x','y','z','roll','pitch','yaw','vx','vy','vz','wx','wy','wz'};
        units = {'mm','mm','mm','deg','deg','deg','mm/s','mm/s','mm/s','deg/s','deg/s','deg/s'};
        scales = [1e3 1e3 1e3 180/pi 180/pi 180/pi 1e3 1e3 1e3 180/pi 180/pi 180/pi];
    else
        error('plot_trajectory_overlay:bad_dim', ...
              'baseline_data.x has %d rows (expected 10 or 12)', nxb);
    end

    % --- Align time vectors --------------------------------------------
    nbf = min(size(Xf_proj,2), size(Xb,2));
    Xf_proj = Xf_proj(:, 1:nbf);
    Xb      = Xb(:, 1:nbf);
    t = (0:(nbf-1)) * dtb * 1e3;   % ms

    % --- Build figure --------------------------------------------------
    fig = figure('Name', ttl, 'Position', [50 50 1200 900]);
    n_states = nxb;
    rows = ceil(n_states / 3);
    cols = 3;

    for s = 1:n_states
        ax = subplot(rows, cols, s);
        hold(ax, 'on'); grid(ax, 'on');
        plot(ax, t, Xb(s,:) * scales(s),  'k-',  'LineWidth', 1.2);
        plot(ax, t, Xf_proj(s,:) * scales(s), '--', 'LineWidth', 1.0, 'Color', [0.85 0.30 0.20]);
        xlabel(ax, 't [ms]');
        ylabel(ax, sprintf('%s [%s]', state_names{s}, units{s}));
        title(ax, state_names{s});
        if s == 1
            legend(ax, {'baseline','framework'}, 'Location', 'best');
        end
    end

    sgtitle(ttl);

    pub_style('Width', 240, 'Height', 180, 'FontSize', 9);

    if ~isempty(sp)
        savefig(fig, sp);
    end
end
