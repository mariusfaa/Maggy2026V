%% --- PLOT NMPC SIMULATION RESULTS ---
%clear all; clc;

load('nmpc_results.mat');

% t     = sim_data.t;
% x     = sim_data.x;      % 12 x (sim_steps+1)
% u     = sim_data.u;      % 4  x  sim_steps
% xEq   = sim_data.xEq;
% uEq   = sim_data.uEq;
% dt    = sim_data.dt;

% Time vector for inputs (zero-order hold — applied at start of each step)
t_u = t(1:end-1);

% Convert to more readable units
pos_scale  = 1e2;   % m  -> cm
ang_scale  = 180/pi; % rad -> deg
pos_unit   = 'cm';
ang_unit   = 'deg';

xEq_pos = xEq(1:3) * pos_scale;
xEq_ang = xEq(4:6) * ang_scale;

%% ---- FIGURE 1: Position & Orientation ----
figure(1); clf;
set(gcf, 'Name', 'States vs Reference', 'Position', [100 100 900 700]);

state_labels = {'x', 'y', 'z', '\alpha (roll)', '\beta (pitch)', '\gamma (yaw)'};
ref_vals     = [xEq_pos; xEq_ang];
colors       = lines(6);

for i = 1:6
    subplot(3, 2, i);
    hold on; grid on; box on;
    if i <= 3
        data = x(i,:) * pos_scale;
        ylab = pos_unit;
        
        if i == 3
            ylim([2,4]);
        else
            ylim([-2,2])
        end
    else
        data = x(i,:) * ang_scale;
        ylab = ang_unit;
    end

    % Reference line
    yline(ref_vals(i), '--k', 'LineWidth', 1.2, 'DisplayName', 'Reference');

    % State trajectory
    plot(t, data, 'Color', colors(i,:), 'LineWidth', 1.8, 'DisplayName', state_labels{i});

    xlabel('Time (s)');
    ylabel(ylab);
    title(state_labels{i}, 'Interpreter', 'tex');
    legend('Location', 'best');
    xlim([t(1), t(end)]);
end

sgtitle('Position and Orientation vs Reference', 'FontSize', 14, 'FontWeight', 'bold');

%% ---- FIGURE 2: Control Inputs ----
figure(2); clf;
set(gcf, 'Name', 'Control Inputs', 'Position', [1020 100 750 600]);

input_colors = lines(4);

for i = 1:4
    subplot(4, 1, i);
    hold on; grid on; box on;

    % Equilibrium current reference
    yline(uEq(i), '--k', 'LineWidth', 1.2, 'DisplayName', 'u_{eq}');

    % Input as zero-order hold stairs
    stairs(t_u, u(i,:), 'Color', input_colors(i,:), 'LineWidth', 1.8, ...
        'DisplayName', sprintf('u_%d', i));

    % Constraint limits
    yline( 1, ':r', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    yline(-1, ':r', 'LineWidth', 1.0, 'HandleVisibility', 'off');

    ylabel('A');
    title(sprintf('Solenoid %d current', i));
    legend('Location', 'best');
    xlim([t_u(1), t_u(end)]);
    ylim([-1.2, 1.2]);
end

xlabel('Time (s)');
sgtitle('Control Inputs (solenoid currents)', 'FontSize', 14, 'FontWeight', 'bold');