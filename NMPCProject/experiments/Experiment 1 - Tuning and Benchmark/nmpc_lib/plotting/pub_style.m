function pub_style(varargin)
% PUB_STYLE  Apply publication-quality styling to the current figure / axes.
%
%   pub_style()                    use defaults
%   pub_style('Width',89)           set figure width in mm (one-column default)
%   pub_style('Width',183)          two-column width
%   pub_style('FontSize',9)
%
% Sets:
%   - Figure size (mm), background color (white)
%   - Axes font, line widths, tick directions
%   - Grid on with subtle line
%   - Color-blind-safe palette for the color order
%
% The current figure must already exist. Apply BEFORE final adjustments to
% titles / labels so they pick up the chosen font size.

    p = inputParser();
    p.addParameter('Width',     183,    @(x) isscalar(x) && x>0);   % mm; 89 single, 183 double
    p.addParameter('Height',    [],     @(x) isempty(x) || (isscalar(x) && x>0));
    p.addParameter('FontSize',  10,     @(x) isscalar(x) && x>0);
    p.addParameter('LineWidth', 1.0,    @(x) isscalar(x) && x>0);
    p.parse(varargin{:});
    opt = p.Results;

    fig = gcf;
    set(fig, 'Color', 'w');
    set(fig, 'Units', 'centimeters');
    pos = get(fig, 'Position');
    width_cm = opt.Width / 10;
    if isempty(opt.Height)
        height_cm = width_cm * 0.6;
    else
        height_cm = opt.Height / 10;
    end
    set(fig, 'Position', [pos(1) pos(2) width_cm height_cm]);
    set(fig, 'PaperPositionMode', 'auto');

    % Color-blind-safe palette (Wong 2011 with tweaks for legibility).
    palette = [ ...
        0.00 0.45 0.70;   % blue
        0.90 0.62 0.00;   % orange
        0.00 0.62 0.45;   % bluish-green
        0.80 0.40 0.00;   % vermillion
        0.35 0.70 0.90;   % sky blue
        0.95 0.90 0.25;   % yellow
        0.80 0.60 0.70;   % reddish purple
        0.40 0.40 0.40];  % gray

    axs = findall(fig, 'Type', 'axes');
    for k = 1:numel(axs)
        ax = axs(k);
        set(ax, 'FontSize',     opt.FontSize, ...
                'LineWidth',    0.7, ...
                'TickDir',      'out', ...
                'TickLength',   [0.008 0.008], ...
                'XGrid',        'on', ...
                'YGrid',        'on', ...
                'GridAlpha',    0.15, ...
                'Box',          'off', ...
                'ColorOrder',   palette);
        % Apply line width to existing lines.
        lh = findall(ax, 'Type', 'line');
        for j = 1:numel(lh)
            if get(lh(j),'LineWidth') < opt.LineWidth
                set(lh(j), 'LineWidth', opt.LineWidth);
            end
        end
    end
end
