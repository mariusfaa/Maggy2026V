%% hardwareNMPCController.m
% =========================================================================
%  Host-side NMPC controller for the MagLev v4.3 hardware.
%
%  Communicates with a Teensy 4.1 running TeensyNMPC_IO.slx via USB serial.
%  The Teensy reads sensors and drives solenoids; this script runs the NMPC.
%
%  REDUCED-ORDER MODEL: 10 states (yaw angle gamma and yaw rate wz removed)
%
%  State vector (10x1):
%    x(1:3)  = [x, y, z]          position
%    x(4:5)  = [alpha, beta]      roll, pitch
%    x(6:8)  = [vx, vy, vz]       linear velocity
%    x(9:10) = [wx, wy]           angular velocity (body frame, x & y)
%
%  Usage:
%    1. Deploy TeensyNMPC_IO.slx to the Teensy 4.1
%    2. Set serialPortName below (use serialportlist() to find it)
%    3. Optionally set stateEstimatorFcn to your EKF/UKF
%    4. Run this script
%
%  Without a state estimator (stateEstimatorFcn = []), the script runs in
%  open-loop logging mode: receives and logs sensor data, sends zero current.
%
%  Author: Marius Jullum Faanes
%  Date:   05.04.2026
% =========================================================================

%% --- PROJECT SETUP ---
clearvars; clc;

% Windows root
acados_root  = 'C:\Users\mariujf\acados';
project_root = 'C:\Users\mariujf\MagLevTbx-main\Controller';

setenv('ACADOS_SOURCE_DIR',        acados_root);
setenv('ENV_ACADOS_INSTALL_DIR',   acados_root);
setenv('ACADOS_INSTALL_DIR',       acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

% Add hardware utilities to path
addpath(fileparts(mfilename('fullpath')));

import casadi.*

%% --- MODEL SETUP (reduced order: 10 states) ---
nx_full = 12;
nx      = 10;
nu      = 4;

x_full  = SX.sym('x_full', nx_full);
u_sym   = SX.sym('u', nu);

parameters_maggy_V4;
correctionFactorFast   = computeSolenoidRadiusCorrectionFactor(params, 'fast');
paramsFast             = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

%% --- BUILD FULL-ORDER CASADI FUNCTION ---
f_expl_full = maglevSystemDynamicsCasADi(x_full, u_sym, paramsFast);
f_func_full = casadi.Function('f_full', {x_full, u_sym}, {f_expl_full});

%% --- HARDCODED EQUILIBRIUM ---
zEq = 0.0301191833;
uEq = zeros(nu, 1);
xEq = [0; 0; zEq; zeros(7, 1)];

fprintf('Equilibrium: z = %.10f m, u = [0, 0, 0, 0]\n', zEq);

%% --- BUILD REDUCED-ORDER DYNAMICS ---
x_r    = SX.sym('x_r', nx);
xdot_r = SX.sym('xdot_r', nx);

x_full_from_r = [x_r(1:5); 0; x_r(6:8); x_r(9:10); 0];

dx_full  = f_func_full(x_full_from_r, u_sym);
f_expl_r = [dx_full(1:5); dx_full(7:11)];

%% --- OCP SETUP ---
N      = 10;
Tf     = 0.3;
dt_mpc = Tf / N;   % 0.03 s  (budget: ~15ms NMPC + ~10ms EKF + margin)

Q = diag([1e2, 1e2, 1e3, ...
          1e3, 1e3, ...
          1e1, 1e1, 1e1, ...
          1e1, 1e1]);
R = eye(nu) * 1.0;

ocp = AcadosOcp();
ocp.model.name        = 'maglev_nmpc_reduced';
ocp.model.x           = x_r;
ocp.model.u           = u_sym;
ocp.model.xdot        = xdot_r;
ocp.model.f_impl_expr = xdot_r - f_expl_r;

ocp.solver_options.N_horizon             = N;
ocp.solver_options.tf                    = Tf;
ocp.solver_options.integrator_type       = 'IRK';
ocp.solver_options.sim_method_num_stages = 2;
ocp.solver_options.sim_method_num_steps  = 5;
ocp.solver_options.nlp_solver_type       = 'SQP_RTI';
ocp.solver_options.nlp_solver_tol_stat   = 1e-4;
ocp.solver_options.nlp_solver_tol_eq     = 1e-4;
ocp.solver_options.nlp_solver_tol_ineq   = 1e-4;
ocp.solver_options.nlp_solver_tol_comp   = 1e-4;
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_iter_max    = 200;
ocp.solver_options.qp_solver_warm_start  = 1;
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';
ocp.solver_options.regularize_method     = 'CONVEXIFY';

ocp.cost.cost_type   = 'NONLINEAR_LS';
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.cost.W           = blkdiag(Q, R);
ocp.cost.W_0         = blkdiag(Q, R);
ocp.cost.W_e         = Q * 50;

ocp.model.cost_y_expr   = [x_r; u_sym];
ocp.model.cost_y_expr_0 = [x_r; u_sym];
ocp.model.cost_y_expr_e = x_r;

ocp.cost.yref   = [xEq; uEq];
ocp.cost.yref_0 = [xEq; uEq];
ocp.cost.yref_e = xEq;

ocp.constraints.idxbu = 0:nu-1;
ocp.constraints.lbu   = -1 * ones(nu, 1);
ocp.constraints.ubu   =  1 * ones(nu, 1);

ocp.constraints.idxbx  = [0, 1, 2, 3, 4];
ocp.constraints.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
ocp.constraints.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
ocp.constraints.idxsbx = 0:4;

n_sbx = 5;
ocp.cost.Zl = 1e3 * ones(n_sbx, 1);
ocp.cost.Zu = 1e3 * ones(n_sbx, 1);
ocp.cost.zl = 1e2 * ones(n_sbx, 1);
ocp.cost.zu = 1e2 * ones(n_sbx, 1);

ocp.constraints.x0 = xEq;

%% --- BUILD OCP SOLVER ---
% Always rebuild when Tf/N changed — clear stale solver
if exist('ocp_solver', 'var')
    try, delete(ocp_solver); catch, end
    clear ocp_solver
end
fprintf('\n--- Building acados OCP solver (reduced, nx=%d, Tf=%.2f, N=%d) ---\n', nx, Tf, N);
ocp_solver = AcadosOcpSolver(ocp);

%% --- SERIAL PORT SETUP ---
% Set your serial port name here. Use serialportlist() to find it.
serialPortName = "COM5";   % <-- CHANGE THIS
baudRate = 115200;          % Ignored for Teensy USB (native speed)

fprintf('\n--- Opening serial port %s ---\n', serialPortName);

% Release any stale serialport handle from a previous failed run.
% Without this, MATLAB still holds COM4 open and serialportlist("available")
% hides it until the Teensy is physically reset.
if exist('sp', 'var')
    try, delete(sp); catch, end %#ok<NOCOM>
    clear sp
end

% Wait for the port to appear in serialportlist("available") — handles
% post-deploy enumeration lag so the user doesn't need to press reset.
portWaitTimeout = 5.0;
t0 = tic;
while ~ismember(serialPortName, serialportlist("available"))
    if toc(t0) > portWaitTimeout
        error(['Serial port %s not available. Press the Teensy reset ' ...
               'button and rerun.'], serialPortName);
    end
    pause(0.1);
end

sp = serialport(serialPortName, baudRate);
configureTerminator(sp, 0);
flush(sp);
pause(1.5);  % give Teensy DTR-gated send path time to emit a fresh frame

% Register emergency stop on cleanup (Ctrl-C, error, or normal exit)
cleanupObj = onCleanup(@() emergencyStop(sp));

% Packet specifications (must match TeensyNMPC_IO.slx exactly)
packetSpec_meas = {'3*double', 'single', 'single', 'single', 'single'};
packetSpec_cmd  = {'4*int16'};

% Force MATLAB to reload the (recently fixed) getPacketInfo from disk.
clear getPacketInfo

% Compute expected packet size for buffer draining
packetInfo_meas   = getPacketInfo(packetSpec_meas);
expectedPacketLen = packetInfo_meas.byteSize + ceil(packetInfo_meas.byteSize / 254) + 1;

% Synchronize: discard initial corrupted frames (host connects mid-stream).
% Strategy: (1) wait for any inbound bytes (detects silent Teensy / no DTR),
% (2) locate a null delimiter, (3) read the next full frame between two nulls
% and decode it. On failure, delete sp before rethrowing so no stale handle
% leaks into the next run.
fprintf('Synchronizing serial stream...\n');
try
    % (1) Wait for the first inbound bytes
    dataWaitTimeout = 3.0;
    tData = tic;
    while sp.NumBytesAvailable == 0
        if toc(tData) > dataWaitTimeout
            error(['No data received from Teensy within %.1f s. ' ...
                   'Verify TeensyNMPC_IO.slx is deployed and running, ' ...
                   'and that UsbSerialPacketSend has "Wait for DTR" ' ...
                   'behaving correctly.'], dataWaitTimeout);
        end
        pause(0.05);
    end
    fprintf('Received first bytes (%d available). Aligning...\n', sp.NumBytesAvailable);

    % (2) Read until we hit a null byte → we are now at a frame boundary
    alignStart = tic;
    while true
        if toc(alignStart) > 2.0
            error('Timed out searching for null-byte frame delimiter.');
        end
        if sp.NumBytesAvailable == 0
            pause(0.01);
            continue;
        end
        if read(sp, 1, 'uint8') == 0
            break;
        end
    end

    % (3) Try to parse up to N frames; a valid decode means we are synced
    syncAttempts = 50;
    synced = false;
    lastErr = '';
    for s = 1:syncAttempts
        try
            % Read one full frame (bytes up to next null) with a cap to
            % avoid runaway reads if framing is broken.
            frame = uint8([]);
            frameStart = tic;
            while true
                if toc(frameStart) > 1.0
                    error('Frame read timeout.');
                end
                if sp.NumBytesAvailable == 0
                    pause(0.005);
                    continue;
                end
                b = read(sp, 1, 'uint8');
                if b == 0
                    break;
                end
                frame(end+1) = uint8(b); %#ok<SAGROW>
                if numel(frame) > 2 * expectedPacketLen
                    error('Frame exceeded expected length (%d).', expectedPacketLen);
                end
            end

            % Attempt to decode and unpack
            decoded = cobsDecode(frame);
            dataUnpack(decoded, packetInfo_meas);  % throws on size mismatch

            fprintf('Serial synchronized after %d frame(s).\n', s);
            synced = true;
            break;
        catch innerME
            lastErr = innerME.message;
        end
    end
    if ~synced
        error(['Failed to synchronize serial after %d attempts. ' ...
               'Last error: %s'], syncAttempts, lastErr);
    end
catch ME
    try, delete(sp); catch, end %#ok<NOCOM>
    clear sp
    rethrow(ME);
end

% Drain any remaining stale packets
while sp.NumBytesAvailable > expectedPacketLen
    try
        serialPacketReceive(sp, packetSpec_meas, true, true);
    catch
        if sp.NumBytesAvailable > 0
            read(sp, 1, 'uint8');
        end
    end
end

%% --- STATE ESTIMATOR SETUP ---
% Choose observer: 'EKF', 'MHE', or '' for open-loop logging mode.
%
% Function signature (both observers):
%   x_est = stateEstimatorFcn(y_mag, i_meas, u_prev, dt)
%
% where:
%   y_mag   = double [3x1]  magnetic field [Bx, By, Bz] from sensor (in mT)
%   i_meas  = double [4x1]  measured solenoid currents (in A)
%   u_prev  = double [4x1]  previous NMPC command (normalized [-1, 1])
%   dt      = double         time step (s)
%   x_est   = double [10x1] estimated state in reduced-order form
observerType = 'EKF';   % <-- 'EKF', 'MHE', or '' for open-loop

if ~isempty(observerType)
    addpath(fullfile(fileparts(mfilename('fullpath')), 'observers'));
    stateEstimatorFcn = initObserver(observerType, paramsFast, ...
        f_expl_r, x_r, u_sym, xEq, uEq, dt_mpc);
else
    stateEstimatorFcn = [];
end

%% --- INITIALIZE TRAJECTORY ---
for k = 0:N
    ocp_solver.set('x', xEq, k);
end
for k = 0:N-1
    ocp_solver.set('u', uEq, k);
end

%% --- CONTROL LOOP ---
control_steps = 1000;       % Total number of NMPC steps
n_meas        = 7;          % 3 mag + 4 current

plot_y    = zeros(n_meas, control_steps);
plot_x    = zeros(nx, control_steps);
plot_u    = zeros(nu, control_steps);
plot_stat = zeros(1, control_steps);
plot_sqp  = zeros(1, control_steps);
plot_time = zeros(1, control_steps);
plot_loop = zeros(1, control_steps);

u_prev = uEq;

closedLoop = ~isempty(observerType) && isstruct(stateEstimatorFcn);
if closedLoop
    fprintf('\n--- Starting CLOSED-LOOP NMPC control (%d steps, %.2f s) ---\n', ...
        control_steps, control_steps * dt_mpc);
else
    fprintf('\n--- Starting OPEN-LOOP logging mode (%d steps, %.2f s) ---\n', ...
        control_steps, control_steps * dt_mpc);
    fprintf('    (stateEstimatorFcn is empty — sending zero current)\n');
end

% Send initial zero command
serialPacketSend(sp, {int16([0; 0; 0; 0])}, true, true);

for i = 1:control_steps
    t_loop = tic;

    % --- 1. Drain serial buffer to get latest measurement ---
    while sp.NumBytesAvailable > expectedPacketLen
        try
            serialPacketReceive(sp, packetSpec_meas, true, true);
        catch
            if sp.NumBytesAvailable > 0, read(sp, 1, 'uint8'); end
        end
    end
    meas = serialPacketReceive(sp, packetSpec_meas, true, true);

    y_mag  = meas{1}(:);                                           % double [3x1]
    i_hw   = double([meas{2}; meas{3}; meas{4}; meas{5}]);        % double [4x1] in hardware order [+X,-X,+Y,-Y]
    i_meas = i_hw([1, 3, 2, 4]);                                   % permute to model order [+X,+Y,-X,-Y]

    plot_y(:, i) = [y_mag; i_meas];

    % --- 2. State estimation ---
    if closedLoop
        x_est = stateEstimatorFcn.step(y_mag, i_meas, u_prev, dt_mpc);

        % --- 3. NMPC solve ---
        ocp_solver.set('constr_x0', x_est);
        ocp_solver.solve();

        status   = ocp_solver.get('status');
        sqp_iter = ocp_solver.get('sqp_iter');
        time_tot = ocp_solver.get('time_tot');

        if status == 0
            % QP succeeded — use the solution
            u_applied = ocp_solver.get('u', 0);
            u_applied = max(min(u_applied, 1), -1);

            % Warm-start shift for next step
            x_traj = ocp_solver.get('x');
            u_traj = ocp_solver.get('u');
            x_traj_shift = [x_traj(:, 2:end), xEq];
            u_traj_shift = [u_traj(:, 2:end), uEq];
            for k = 0:N
                ocp_solver.set('x', x_traj_shift(:, k+1), k);
            end
            for k = 0:N-1
                ocp_solver.set('u', u_traj_shift(:, k+1), k);
            end
        else
            % QP failed — do NOT use the solution. Send safe zero command
            % and reset both NMPC warm-start and EKF to equilibrium.
            u_applied = uEq;
            for k = 0:N
                ocp_solver.set('x', xEq, k);
            end
            for k = 0:N-1
                ocp_solver.set('u', uEq, k);
            end
            % Reset observer to prevent corrupted estimates from persisting
            stateEstimatorFcn.reset();
        end

        % --- 4. Send command ---
        cmd = nmpcToDriverCommand(u_applied);
        serialPacketSend(sp, {cmd}, true, true);

        % --- 5. Log ---
        plot_x(:, i)  = x_est;
        plot_u(:, i)  = u_applied;
        plot_stat(i)  = status;
        plot_sqp(i)   = sqp_iter;
        plot_time(i)  = time_tot;
        u_prev        = u_applied;

    else
        % Open-loop: send zero current
        serialPacketSend(sp, {int16([0; 0; 0; 0])}, true, true);
        plot_u(:, i) = zeros(nu, 1);
    end

    % --- 7. Timing ---
    elapsed = toc(t_loop);
    plot_loop(i) = elapsed;

    if elapsed < dt_mpc
        pause(dt_mpc - elapsed);
    elseif closedLoop
        fprintf('WARNING: loop overrun at step %d (%.1f ms > %.1f ms)\n', ...
            i, elapsed * 1000, dt_mpc * 1000);
    end

    % --- 8. Print progress ---
    if closedLoop && (i <= 10 || mod(i, 20) == 0 || i == control_steps)
        fprintf('Step %3d: st=%d, sqp=%2d, t=%5.1fms, u=[%+.3f %+.3f %+.3f %+.3f], z=%.4fmm\n', ...
            i, status, sqp_iter, time_tot * 1000, ...
            u_applied(1), u_applied(2), u_applied(3), u_applied(4), ...
            x_est(3) * 1e3);
    elseif ~closedLoop && (i <= 5 || mod(i, 50) == 0 || i == control_steps)
        fprintf('Step %3d: Bx=%.2f By=%.2f Bz=%.2f mT, I=[%.3f %.3f %.3f %.3f] A\n', ...
            i, y_mag(1), y_mag(2), y_mag(3), ...
            i_meas(1), i_meas(2), i_meas(3), i_meas(4));
    end
end

%% --- SEND FINAL ZERO COMMAND ---
serialPacketSend(sp, {int16([0; 0; 0; 0])}, true, true);

%% --- STATISTICS ---
n_actual = control_steps;
fprintf('\n--- Session summary ---\n');
fprintf('  Mode:             %s\n', ternary(closedLoop, 'Closed-loop NMPC', 'Open-loop logging'));
fprintf('  Steps completed:  %d / %d\n', n_actual, control_steps);
fprintf('  Avg loop time:    %.2f ms\n', mean(plot_loop(1:n_actual)) * 1000);
fprintf('  Max loop time:    %.2f ms\n', max(plot_loop(1:n_actual)) * 1000);

if closedLoop
    fprintf('  Solver status:    %d OK, %d max-iter, %d QP-fail\n', ...
        sum(plot_stat(1:n_actual) == 0), sum(plot_stat(1:n_actual) == 2), ...
        sum(plot_stat(1:n_actual) == 4));
    fprintf('  Avg SQP iter:     %.1f\n', mean(plot_sqp(1:n_actual)));
    fprintf('  Avg solve time:   %.2f ms\n', mean(plot_time(1:n_actual)) * 1000);
    fprintf('  Max solve time:   %.2f ms\n', max(plot_time(1:n_actual)) * 1000);
end

%% --- PLOTS ---
t_steps = (0:n_actual-1) * dt_mpc;

if closedLoop
    % --- Closed-loop plots (state + control + solver) ---
    fig = figure('Name', 'NMPC Hardware Control', 'Position', [50 50 1400 900]);

    subplot(3, 2, 1); hold on; grid on;
    plot(t_steps * 1e3, plot_x(1, 1:n_actual) * 1e3, 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_x(2, 1:n_actual) * 1e3, 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_x(3, 1:n_actual) * 1e3, 'LineWidth', 1.2);
    yline(xEq(3) * 1e3, '--k', 'z_{eq}');
    xlabel('Time [ms]'); ylabel('[mm]');
    legend('x', 'y', 'z', 'Location', 'best');
    title('Estimated Position');

    subplot(3, 2, 2); hold on; grid on;
    plot(t_steps * 1e3, rad2deg(plot_x(4, 1:n_actual)), 'LineWidth', 1.2);
    plot(t_steps * 1e3, rad2deg(plot_x(5, 1:n_actual)), 'LineWidth', 1.2);
    xlabel('Time [ms]'); ylabel('[deg]');
    legend('\alpha (roll)', '\beta (pitch)', 'Location', 'best');
    title('Estimated Orientation');

    subplot(3, 2, 3); hold on; grid on;
    stairs(t_steps * 1e3, plot_u(1, 1:n_actual), 'LineWidth', 1.2);
    stairs(t_steps * 1e3, plot_u(2, 1:n_actual), 'LineWidth', 1.2);
    stairs(t_steps * 1e3, plot_u(3, 1:n_actual), 'LineWidth', 1.2);
    stairs(t_steps * 1e3, plot_u(4, 1:n_actual), 'LineWidth', 1.2);
    yline(-1, '--k'); yline(1, '--k');
    xlabel('Time [ms]'); ylabel('[A]');
    legend('u_1', 'u_2', 'u_3', 'u_4', 'Location', 'best');
    title('Solenoid Currents (NMPC)');

    subplot(3, 2, 4); hold on; grid on;
    plot(t_steps * 1e3, plot_y(1, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(2, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(3, 1:n_actual), 'LineWidth', 1.2);
    xlabel('Time [ms]'); ylabel('[mT]');
    legend('B_x', 'B_y', 'B_z', 'Location', 'best');
    title('Magnetic Sensor');

    subplot(3, 2, 5); hold on; grid on;
    plot(t_steps * 1e3, plot_y(4, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(5, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(6, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(7, 1:n_actual), 'LineWidth', 1.2);
    xlabel('Time [ms]'); ylabel('[A]');
    legend('I_1', 'I_2', 'I_3', 'I_4', 'Location', 'best');
    title('Measured Currents');

    subplot(3, 2, 6); hold on; grid on;
    yyaxis left;
    stem(t_steps * 1e3, plot_stat(1:n_actual), 'filled', 'MarkerSize', 3);
    ylabel('Status (0=OK)');
    yyaxis right;
    plot(t_steps * 1e3, plot_time(1:n_actual) * 1000, 'LineWidth', 1.0);
    ylabel('Solve time [ms]');
    xlabel('Time [ms]');
    title('Solver Performance');

    sgtitle('NMPC Hardware Control — MagLev v4.3');

else
    % --- Open-loop plots (raw sensor data) ---
    fig = figure('Name', 'Open-Loop Sensor Logging', 'Position', [50 50 1200 700]);

    subplot(2, 1, 1); hold on; grid on;
    plot(t_steps * 1e3, plot_y(1, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(2, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(3, 1:n_actual), 'LineWidth', 1.2);
    xlabel('Time [ms]'); ylabel('[mT]');
    legend('B_x', 'B_y', 'B_z', 'Location', 'best');
    title('Magnetic Sensor Readings');

    subplot(2, 1, 2); hold on; grid on;
    plot(t_steps * 1e3, plot_y(4, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(5, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(6, 1:n_actual), 'LineWidth', 1.2);
    plot(t_steps * 1e3, plot_y(7, 1:n_actual), 'LineWidth', 1.2);
    xlabel('Time [ms]'); ylabel('[A]');
    legend('I_1', 'I_2', 'I_3', 'I_4', 'Location', 'best');
    title('Current Sensor Readings');

    sgtitle('Open-Loop Sensor Logging — MagLev v4.3');
end

%% --- SAVE ---
hw_data              = struct();
hw_data.t            = t_steps;
hw_data.y            = plot_y;
hw_data.x            = plot_x;
hw_data.u            = plot_u;
hw_data.status       = plot_stat;
hw_data.sqp_iter     = plot_sqp;
hw_data.solve_time   = plot_time;
hw_data.loop_time    = plot_loop;
hw_data.xEq          = xEq;
hw_data.uEq          = uEq;
hw_data.dt           = dt_mpc;
hw_data.N            = N;
hw_data.Tf           = Tf;
hw_data.Q            = Q;
hw_data.R            = R;
hw_data.closedLoop   = closedLoop;

save('hardware_nmpc_results.mat', '-struct', 'hw_data');
fprintf('\nResults saved to hardware_nmpc_results.mat\n');

%% --- LOCAL HELPERS ---
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
