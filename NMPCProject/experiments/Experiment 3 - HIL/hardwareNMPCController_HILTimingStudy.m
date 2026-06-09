%% hardwareNMPCController_HILTimingStudy.m
% =========================================================================
%  High-Fidelity HIL Timing Characterization for MagLev v4.3
%
%  PURPOSE
%  -------
%  This script performs deterministic Hardware-in-the-Loop (HIL)
%  timing characterization of the acados-based NMPC controller running on
%  the host PC while communicating with the Teensy 4.1 over USB serial.
%
%  Instead of using a live observer, the controller replays a previously
%  simulated trajectory generated offline:
%
%      nmpc_results_reduced.mat
%
%  At iteration i:
%
%      x_current = sim_data.x(:,i)
%
%  is injected directly into the NMPC solver:
%
%      ocp_solver.set('constr_x0', x_current)
%
%  This isolates computational and communication timing performance from
%  observer uncertainty and estimator latency.
%
%  OBJECTIVE
%  ---------
%  Quantify whether the full HIL stack can satisfy a strict:
%
%      100 Hz hard real-time deadline (10 ms)
%
%  including:
%      - NMPC solve time
%      - USB serial communication
%      - MATLAB execution overhead
%      - warm-start shifting
%
%  FEATURES
%  --------
%   * Trajectory playback (deterministic state injection)
%   * Batch benchmarking across multiple horizons
%   * Precise tic/toc instrumentation
%   * WCET analysis
%   * Jitter analysis
%   * Deadline miss statistics
%   * Automatic summary generation
%   * Comparative plots
%
%  IMPORTANT
%  ---------
%  The following hardware plumbing is intentionally preserved EXACTLY:
%
%      - COBS framing
%      - serialPacketReceive()
%      - serialPacketSend()
%      - getPacketInfo()
%      - hardware current remapping
%
%  so the Teensy firmware requires NO modification.
%
%  Author: Marius Jullum Faanes
%  Refactored for thesis timing study
% =========================================================================

%% ------------------------------------------------------------------------
%  PROJECT SETUP
% -------------------------------------------------------------------------
clearvars;
clc;
close all;

%% --- USER CONFIGURATION -----------------------------------------------

% Horizons to benchmark
N_test_list = [5 10 15];

% Real-time deadline
deadline_s = 0.010;       % 10 ms = 100 Hz

% Use only first N samples from trajectory (optional)
maxPlaybackSteps = inf;

% Save directory
resultsDir = 'hil_timing_results';

if ~exist(resultsDir, 'dir')
    mkdir(resultsDir);
end

%% ------------------------------------------------------------------------
%  LOAD TRAJECTORY PLAYBACK DATA
% -------------------------------------------------------------------------
fprintf('\nLoading trajectory playback data...\n');

simData = load('nmpc_results_reduced.mat');

xPlayback = simData.x;
uEq       = simData.uEq;
xEq       = simData.xEq;

nx = size(xPlayback,1);
nu = length(uEq);

nPlaybackSteps = size(xPlayback,2);

if isfinite(maxPlaybackSteps)
    nPlaybackSteps = min(nPlaybackSteps, maxPlaybackSteps);
end

fprintf('Loaded %d playback states.\n', nPlaybackSteps);

%% ------------------------------------------------------------------------
%  PATH SETUP
% -------------------------------------------------------------------------
% --- Enter your own paths here ---
% acados_root  : path to your local acados installation
% project_root : path to the Controller folder
if ispc
    acados_root  = '';   % e.g. 'C:\Users\you\acados'
    project_root = '';   % e.g. 'C:\Users\you\MagLevTbx\Controller'
else
    acados_root  = fullfile(getenv('HOME'), 'acados');
    project_root = fullfile(getenv('HOME'), 'MagLevTbx-main', 'Controller');
end

setenv('ACADOS_SOURCE_DIR',      acados_root);
setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
setenv('ACADOS_INSTALL_DIR',     acados_root);

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external', 'jsonlab'));
addpath(fullfile(acados_root, 'external', 'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

addpath(fileparts(mfilename('fullpath')));

toolbox_root = fileparts(project_root);

addpath(fullfile(toolbox_root, 'Matlab', 'SerialPacketSendReceive'));
addpath(fullfile(toolbox_root, 'Matlab', 'CobsEncodeDecode'));
addpath(fullfile(toolbox_root, 'Matlab', 'DataPackUnpack'));

import casadi.*

%% ------------------------------------------------------------------------
%  MODEL SETUP
% -------------------------------------------------------------------------
fprintf('\nBuilding reduced-order model...\n');

nx_full = 12;

x_full = SX.sym('x_full', nx_full);
u_sym  = SX.sym('u', nu);

parameters_maggy_V4;

correctionFactorFast   = computeSolenoidRadiusCorrectionFactor(params,'fast');

paramsFast             = params;
paramsFast.solenoids.r = correctionFactorFast * paramsFast.solenoids.r;

f_expl_full = maglevSystemDynamicsCasADi(x_full, u_sym, paramsFast);

f_func_full = casadi.Function( ...
    'f_full', ...
    {x_full, u_sym}, ...
    {f_expl_full});

%% --- Reduced-order dynamics -------------------------------------------

x_r    = SX.sym('x_r', nx);
xdot_r = SX.sym('xdot_r', nx);

x_full_from_r = [x_r(1:5); 0; x_r(6:8); x_r(9:10); 0];

dx_full = f_func_full(x_full_from_r, u_sym);

f_expl_r = [dx_full(1:5); dx_full(7:11)];

%% ------------------------------------------------------------------------
%  SERIAL PORT SETUP
% -------------------------------------------------------------------------
fprintf('\nOpening serial port...\n');

if ispc
    serialPortName = "COM5";
else
    serialPortName = "/dev/ttyACM0";
end

baudRate = 115200;

if exist('sp','var')
    try
        delete(sp);
    catch
    end
    clear sp
end

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

configureTerminator(sp,0);

flush(sp);

pause(1.5);

cleanupObj = onCleanup(@() emergencyStop(sp));

%% ------------------------------------------------------------------------
%  PACKET SPECIFICATION
% -------------------------------------------------------------------------
packetSpec_meas = {'3*double','single','single','single','single'};
packetSpec_cmd  = {'4*int16'};

clear getPacketInfo

packetInfo_meas   = getPacketInfo(packetSpec_meas);

if packetInfo_meas.byteSize ~= 40
    error(['Measurement packet spec resolved to %d bytes, expected 40. ' ...
           'packetSpec_meas=%s, getPacketInfo=%s'], ...
        packetInfo_meas.byteSize, strjoin(string(packetSpec_meas), ', '), ...
        which('getPacketInfo'));
end

expectedPacketLen = ...
    packetInfo_meas.byteSize + ...
    ceil(packetInfo_meas.byteSize/254) + 1;

fprintf('Measurement packet payload: %d bytes, encoded frame: %d bytes + null.\n', ...
    packetInfo_meas.byteSize, expectedPacketLen - 1);

%% ------------------------------------------------------------------------
%  SERIAL SYNCHRONIZATION
% -------------------------------------------------------------------------
fprintf('Synchronizing serial stream...\n');

try

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

    alignStart = tic;

    while true

        if toc(alignStart) > 2.0
            error('Timed out searching for null-byte frame delimiter.');
        end

        if sp.NumBytesAvailable == 0
            pause(0.01);
            continue;
        end

        if read(sp,1,'uint8') == 0
            break;
        end
    end

    syncAttempts = 50;
    synced = false;
    lastErr = '';
    frameLenLog = zeros(1, syncAttempts);
    decodedLenLog = nan(1, syncAttempts);

    for s = 1:syncAttempts

        try

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

                b = read(sp,1,'uint8');

                if b == 0
                    break;
                end

                frame(end+1) = uint8(b); %#ok<SAGROW>

                if numel(frame) > 2 * expectedPacketLen
                    error('Frame exceeded expected length (%d).', expectedPacketLen);
                end
            end

            frameLenLog(s) = numel(frame);

            encodedPacketLen = expectedPacketLen - 1;
            if numel(frame) ~= encodedPacketLen
                error('Encoded frame length %d, expected %d.', ...
                    numel(frame), encodedPacketLen);
            end

            decoded = cobsDecode(frame);
            decodedLenLog(s) = numel(decoded);

            if numel(decoded) ~= packetInfo_meas.byteSize
                error('Decoded payload length %d, expected %d.', ...
                    numel(decoded), packetInfo_meas.byteSize);
            end

            dataUnpack(decoded, packetInfo_meas);

            synced = true;

            fprintf('Serial synchronized after %d frame(s).\n', s);

            break;

        catch innerME
            lastErr = innerME.message;
        end
    end

    if ~synced
        validFrameLens = frameLenLog(frameLenLog > 0);
        validDecodedLens = decodedLenLog(~isnan(decodedLenLog));
        if ~isempty(validFrameLens)
            fprintf('Observed encoded frame lengths during sync: %s\n', ...
                mat2str(unique(validFrameLens)));
        end
        if ~isempty(validDecodedLens)
            fprintf('Observed decoded payload lengths during sync: %s\n', ...
                mat2str(unique(validDecodedLens)));
        end
        error(['Failed to synchronize serial after %d attempts. ' ...
               'Last error: %s'], syncAttempts, lastErr);
    end

catch ME

    try
        delete(sp);
    catch
    end

    clear sp

    rethrow(ME);
end

%% ------------------------------------------------------------------------
%  DRAIN STALE BUFFERS
% -------------------------------------------------------------------------
while sp.NumBytesAvailable > expectedPacketLen

    try
        serialPacketReceive(sp, packetSpec_meas, true, true);
    catch

        if sp.NumBytesAvailable > 0
            read(sp,1,'uint8');
        end
    end
end

%% ------------------------------------------------------------------------
%  BENCHMARK RESULT STRUCTURE
% -------------------------------------------------------------------------
benchmarkResults = struct([]);

%% ========================================================================
%  MAIN BATCH BENCHMARK LOOP
% ========================================================================

for cfgIdx = 1:length(N_test_list)

    %% --------------------------------------------------------------------
    %  CURRENT CONFIGURATION
    % ---------------------------------------------------------------------
    N  = N_test_list(cfgIdx);

    Tf = deadline_s * N;

    dt_mpc = deadline_s;

    fprintf('\n====================================================\n');
    fprintf('Testing configuration %d / %d\n', ...
        cfgIdx, length(N_test_list));

    fprintf('Horizon N = %d\n', N);
    fprintf('Prediction horizon Tf = %.3f s\n', Tf);
    fprintf('====================================================\n');

    %% --------------------------------------------------------------------
    %  BUILD OCP
    % ---------------------------------------------------------------------
    Q = diag([1e2 1e2 1e3 ...
              1e3 1e3 ...
              1e1*0.3 1e1*0.3 1e1*0.3 ...
              1e1*0.3 1e1*0.3]);

    R = eye(nu) * 0.1;

    ocp = AcadosOcp();

    ocp.model.name        = sprintf('maglev_hil_N%d',N);
    ocp.model.x           = x_r;
    ocp.model.u           = u_sym;
    ocp.model.xdot        = xdot_r;
    ocp.model.f_impl_expr = xdot_r - f_expl_r;

    %% --- Solver options -------------------------------------------------

    ocp.solver_options.N_horizon             = N;
    ocp.solver_options.tf                    = Tf;

    ocp.solver_options.integrator_type       = 'IRK';

    ocp.solver_options.sim_method_num_stages = 1;
    ocp.solver_options.sim_method_num_steps  = 1;

    ocp.solver_options.nlp_solver_type       = 'SQP_RTI';

    ocp.solver_options.qp_solver             = ...
        'PARTIAL_CONDENSING_HPIPM';

    ocp.solver_options.qp_solver_warm_start  = 1;

    ocp.solver_options.hessian_approx        = ...
        'GAUSS_NEWTON';

    ocp.solver_options.regularize_method     = ...
        'CONVEXIFY';

    %% --- Cost -----------------------------------------------------------

    ocp.cost.cost_type   = 'NONLINEAR_LS';
    ocp.cost.cost_type_0 = 'NONLINEAR_LS';
    ocp.cost.cost_type_e = 'NONLINEAR_LS';

    ocp.cost.W   = blkdiag(Q,R);
    ocp.cost.W_0 = blkdiag(Q,R);
    ocp.cost.W_e = Q * 3;

    ocp.model.cost_y_expr   = [x_r; u_sym];
    ocp.model.cost_y_expr_0 = [x_r; u_sym];
    ocp.model.cost_y_expr_e = x_r;

    ocp.cost.yref   = [xEq; uEq];
    ocp.cost.yref_0 = [xEq; uEq];
    ocp.cost.yref_e = xEq;

    %% --- Constraints ----------------------------------------------------

    ocp.constraints.idxbu = 0:nu-1;

    ocp.constraints.lbu = -1 * ones(nu,1);
    ocp.constraints.ubu =  1 * ones(nu,1);

    % State constraints (soft) — indices in 10-state vector:
    %   0=x, 1=y, 2=z, 3=alpha, 4=beta
    ocp.constraints.idxbx  = [0, 1, 2, 3, 4];
    ocp.constraints.lbx    = [-0.025; -0.025; 0.015; -0.35; -0.35];
    ocp.constraints.ubx    = [ 0.025;  0.025; 0.055;  0.35;  0.35];
    ocp.constraints.idxsbx = 0:4;
    
    n_sbx = 5;
    ocp.cost.Zl = 1e3 * ones(n_sbx, 1);
    ocp.cost.Zu = 1e3 * ones(n_sbx, 1);
    ocp.cost.zl = 1e3 * ones(n_sbx, 1);
    ocp.cost.zu = 1e3 * ones(n_sbx, 1);

    ocp.constraints.x0 = xEq;

    %% --------------------------------------------------------------------
    %  REBUILD SOLVER
    % ---------------------------------------------------------------------
    fprintf('Building acados solver...\n');

    if exist('ocp_solver','var')

        try
            delete(ocp_solver);
        catch
        end

        clear ocp_solver
    end

    ocp_solver = AcadosOcpSolver(ocp);

    %% --------------------------------------------------------------------
    %  INITIALIZE TRAJECTORY
    % ---------------------------------------------------------------------
    for k = 0:N
        ocp_solver.set('x', xEq, k);
    end

    for k = 0:N-1
        ocp_solver.set('u', uEq, k);
    end

    %% --------------------------------------------------------------------
    %  PREALLOCATE TIMING LOGS
    % ---------------------------------------------------------------------
    t_solve = zeros(1,nPlaybackSteps);

    t_io    = zeros(1,nPlaybackSteps);

    t_total = zeros(1,nPlaybackSteps);

    solver_status = zeros(1,nPlaybackSteps);

    %% --------------------------------------------------------------------
    %  START TEST
    % ---------------------------------------------------------------------
    fprintf('Starting timing benchmark...\n');

    serialPacketSend(sp, {int16([0;0;0;0])}, true, true);

    for i = 1:nPlaybackSteps

        %% ================================================================
        %  TOTAL LOOP TIMER
        % ================================================================
        tLoop = tic;

        %% ================================================================
        %  I/O TIMING
        % ================================================================
        tIO = tic;

        while sp.NumBytesAvailable > expectedPacketLen

            try
                serialPacketReceive(sp, packetSpec_meas, true, true);
            catch

                if sp.NumBytesAvailable > 0
                    read(sp,1,'uint8');
                end
            end
        end

        meas = serialPacketReceive( ...
            sp, ...
            packetSpec_meas, ...
            true, ...
            true);

        t_io_read = toc(tIO);

        %% ================================================================
        %  TRAJECTORY PLAYBACK STATE INJECTION
        % ================================================================
        x_current = xPlayback(:,i);

        %% ================================================================
        %  SOLVER TIMING
        % ================================================================
        tSolve = tic;

        ocp_solver.set('constr_x0', x_current);

        ocp_solver.solve();

        t_solve(i) = toc(tSolve);

        status = ocp_solver.get('status');

        solver_status(i) = status;

        %% ================================================================
        %  EXTRACT CONTROL
        % ================================================================
        if status == 0

            u_applied = ocp_solver.get('u',0);

            u_applied = max(min(u_applied,1),-1);

        else

            u_applied = zeros(nu,1);
        end

        %% ================================================================
        %  WARM START SHIFTING
        % ================================================================
        x_traj = ocp_solver.get('x');
        u_traj = ocp_solver.get('u');

        x_traj_shift = [x_traj(:,2:end), xEq];
        u_traj_shift = [u_traj(:,2:end), uEq];

        for k = 0:N
            ocp_solver.set('x', x_traj_shift(:,k+1), k);
        end

        for k = 0:N-1
            ocp_solver.set('u', u_traj_shift(:,k+1), k);
        end

        %% ================================================================
        %  SERIAL COMMAND TRANSMISSION
        % ================================================================
        tIOwrite = tic;

        cmd = nmpcToDriverCommand(u_applied);

        serialPacketSend(sp, {cmd}, true, true);

        t_io_write = toc(tIOwrite);

        %% ================================================================
        %  TOTAL I/O TIME
        % ================================================================
        t_io(i) = t_io_read + t_io_write;

        %% ================================================================
        %  TOTAL LOOP TIME
        % ================================================================
        t_total(i) = toc(tLoop);

        %% ================================================================
        %  REAL-TIME ENFORCEMENT
        % ================================================================
        if t_total(i) < deadline_s

            pause(deadline_s - t_total(i));

        end

        %% ================================================================
        %  PROGRESS PRINTING
        % ================================================================
        if i <= 5 || mod(i,100) == 0 || i == nPlaybackSteps

            fprintf(['Step %4d | ' ...
                     'solve = %6.3f ms | ' ...
                     'io = %6.3f ms | ' ...
                     'total = %6.3f ms\n'], ...
                     i, ...
                     t_solve(i)*1000, ...
                     t_io(i)*1000, ...
                     t_total(i)*1000);
        end
    end

    %% --------------------------------------------------------------------
    %  FINAL SAFE ZERO COMMAND
    % ---------------------------------------------------------------------
    serialPacketSend(sp, {int16([0;0;0;0])}, true, true);

    %% --------------------------------------------------------------------
    %  STATISTICAL ANALYSIS
    % ---------------------------------------------------------------------
    fprintf('\nComputing timing statistics...\n');

    deadlineMisses = sum(t_total > deadline_s);

    missRate = 100 * deadlineMisses / nPlaybackSteps;

    stats = struct();

    stats.N                     = N;
    stats.Tf                    = Tf;
    stats.deadline_s            = deadline_s;

    stats.avgSolve_ms           = mean(t_solve)*1000;
    stats.wcetSolve_ms          = max(t_solve)*1000;

    stats.avgIO_ms              = mean(t_io)*1000;
    stats.wcetIO_ms             = max(t_io)*1000;

    stats.avgTotal_ms           = mean(t_total)*1000;
    stats.wcetTotal_ms          = max(t_total)*1000;

    stats.jitter_ms             = std(t_total)*1000;

    stats.deadlineMisses        = deadlineMisses;
    stats.deadlineMissRate_pct  = missRate;

    stats.solverFailures        = sum(solver_status ~= 0);

    stats.t_solve               = t_solve;
    stats.t_io                  = t_io;
    stats.t_total               = t_total;

    %% --------------------------------------------------------------------
    %  DISPLAY SUMMARY
    % ---------------------------------------------------------------------
    fprintf('\nRESULT SUMMARY\n');
    fprintf('--------------------------------------------------\n');
    fprintf('Horizon N:                 %d\n', N);

    fprintf('Average solve time:        %.3f ms\n', ...
        stats.avgSolve_ms);

    fprintf('Worst-case solve time:     %.3f ms\n', ...
        stats.wcetSolve_ms);

    fprintf('Average total latency:     %.3f ms\n', ...
        stats.avgTotal_ms);

    fprintf('Worst-case latency:        %.3f ms\n', ...
        stats.wcetTotal_ms);

    fprintf('Jitter (std):              %.3f ms\n', ...
        stats.jitter_ms);

    fprintf('Deadline miss rate:        %.3f %%\n', ...
        stats.deadlineMissRate_pct);

    fprintf('Solver failures:           %d\n', ...
        stats.solverFailures);

    %% --------------------------------------------------------------------
    %  STORE RESULT
    % ---------------------------------------------------------------------
    benchmarkResults(cfgIdx).stats = stats;

    %% --------------------------------------------------------------------
    %  SAVE INDIVIDUAL RESULT
    % ---------------------------------------------------------------------
    save(fullfile(resultsDir, ...
        sprintf('hil_timing_N%d.mat',N)), ...
        'stats');

end

%% ========================================================================
%  COMPARATIVE SUMMARY TABLE
% ========================================================================

fprintf('\n====================================================\n');
fprintf('COMPARATIVE TIMING SUMMARY\n');
fprintf('====================================================\n');

summaryTable = table();

summaryTable.N = N_test_list(:);

summaryTable.WCET_ms = ...
    arrayfun(@(s)s.stats.wcetTotal_ms, benchmarkResults)';

summaryTable.Average_ms = ...
    arrayfun(@(s)s.stats.avgTotal_ms, benchmarkResults)';

summaryTable.Jitter_ms = ...
    arrayfun(@(s)s.stats.jitter_ms, benchmarkResults)';

summaryTable.DeadlineMiss_pct = ...
    arrayfun(@(s)s.stats.deadlineMissRate_pct, benchmarkResults)';

disp(summaryTable);

%% ------------------------------------------------------------------------
%  SAVE SUMMARY
% -------------------------------------------------------------------------
summaryFile = fullfile(resultsDir, ...
    'hil_timing_summary.mat');

save(summaryFile, ...
    'benchmarkResults', ...
    'summaryTable');

fprintf('\nSaved summary results to:\n%s\n', summaryFile);

%% ========================================================================
%  COMPARATIVE PLOTS
% ========================================================================

fprintf('\nGenerating comparative plots...\n');

%% --- WCET Plot ---------------------------------------------------------

fig1 = figure('Name','WCET Comparison');

bar(summaryTable.N, summaryTable.WCET_ms);

hold on;
yline(deadline_s*1000,'--r','10 ms deadline');

xlabel('Prediction Horizon N');
ylabel('Worst-Case Loop Latency [ms]');

title('HIL Worst-Case Execution Time');

grid on;

savefig(fig1, fullfile(resultsDir,'wcet_comparison.fig'));

%% --- Average Latency Plot ---------------------------------------------

fig2 = figure('Name','Average Latency Comparison');

bar(summaryTable.N, summaryTable.Average_ms);

hold on;
yline(deadline_s*1000,'--r','10 ms deadline');

xlabel('Prediction Horizon N');
ylabel('Average Loop Latency [ms]');

title('Average HIL Loop Latency');

grid on;

savefig(fig2, fullfile(resultsDir,'average_latency.fig'));

%% --- Jitter Plot -------------------------------------------------------

fig3 = figure('Name','Jitter Comparison');

bar(summaryTable.N, summaryTable.Jitter_ms);

xlabel('Prediction Horizon N');
ylabel('Jitter [ms]');

title('Loop Timing Jitter');

grid on;

savefig(fig3, fullfile(resultsDir,'jitter_comparison.fig'));

%% --- Deadline Miss Rate ------------------------------------------------

fig4 = figure('Name','Deadline Miss Rate');

bar(summaryTable.N, summaryTable.DeadlineMiss_pct);

xlabel('Prediction Horizon N');
ylabel('Miss Rate [%]');

title('Real-Time Deadline Miss Rate');

grid on;

savefig(fig4, fullfile(resultsDir,'deadline_miss_rate.fig'));

%% ------------------------------------------------------------------------
%  FINAL SAFE SHUTDOWN
% -------------------------------------------------------------------------
serialPacketSend(sp, {int16([0;0;0;0])}, true, true);

fprintf('\n====================================================\n');
fprintf('HIL TIMING STUDY COMPLETE\n');
fprintf('====================================================\n');

%% ========================================================================
%  THESIS NOTES
% ========================================================================
%
%  Timing Definitions:
%
%   t_solve:
%       Time spent exclusively inside:
%
%           ocp_solver.solve()
%
%       Represents pure NMPC computational cost.
%
%
%   t_io:
%       Combined time spent in:
%
%           serialPacketReceive()
%           serialPacketSend()
%
%       Includes:
%           - USB transfer latency
%           - COBS decode/encode
%           - packet unpacking
%
%
%   t_total:
%       End-to-end control-loop latency including:
%
%           - serial receive
%           - state injection
%           - NMPC solve
%           - warm-start shifting
%           - serial transmit
%           - MATLAB overhead
%
%
%  WCET:
%
%       max(t_total)
%
%  Jitter:
%
%       std(t_total)
%
%  Deadline Miss:
%
%       t_total > 10 ms
%
% ========================================================================
