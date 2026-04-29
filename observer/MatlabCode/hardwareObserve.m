clc;
addpath(genpath('~/MSc/MagLevTbx'));
addpath(genpath('~/Documents/Maggy2026V/NMPCProject/Controller/hardware'));

%% --- SERIAL PORT SETUP ---
% Set your serial port name here. Use serialportlist() to find it.
serialPortName = "/dev/ttyACM0";   % <-- CHANGE THIS
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
clear obs;
obs = Observer(2, dt, xLp(1:6), 'normalized', false);

%% lqr

Ixred = [1:3, 6:8];
Kh = Kred(:,Ixred);

serialPacketSend(sp, {int16([0; 0; 0; 0])}, true, true);

%% --- CONTROL LOOP ---
control_steps = 1000;       % Total number of NMPC steps

plot_y    = zeros(3, control_steps);
plot_x    = zeros(6, control_steps);

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

    plot_y(:, i) = y_mag;

    % --- 2. State estimation ---
        x_est = obs.run(i_meas, y_mag);

        % --- 5. Log ---
        plot_x(:, i)  = x_est;

end

%%
tSpan = 0:dt:(control_steps*dt);
figure(1);
clf;

% Subplot 1: XYZ positions
hold on; grid on; box on;
plot(tSpan(1:end-1), plot_x(1:3,:), 'linewidth', 2, 'LineStyle', '-');
xlabel('t (s)');
ylabel('Position (m)');
title('XYZ States');
legend({'x', 'y', 'z'}, 'location', 'best');