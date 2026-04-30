%% TeensyNMPC_IO_Setup.m
% =========================================================================
%  Step-by-step guide to build the TeensyNMPC_IO Simulink model.
%
%  This model runs standalone on the Teensy 4.1 and acts as the I/O layer
%  for the host-side NMPC controller (hardwareNMPCController.m).
%
%  It reads sensors, sends measurements to the host via USB serial, receives
%  current commands from the host, and drives the solenoids.
%
%  Target hardware: MagLev v4.3, Teensy 4.1
% =========================================================================
%
%  PREREQUISITES:
%    - MagLevTbx_Setup has been run (MEX files compiled, paths configured)
%    - Hardware version set to v4.3 via MagLevTbx_HwConfig()
%    - Arduino Support Package installed and configured for Teensy 4.1
%
%  MODEL SETTINGS:
%    - Solver:          Fixed-step, Discrete (no continuous states)
%    - Fixed step size:  0.005  (5 ms)
%    - Hardware board:   Teensy 4.1 (via Arduino Support Package)
%    - Build action:     Build, Deploy & Start
%
%  =========================================================================
%  BLOCK LAYOUT
%  =========================================================================
%
%  The model has two data paths. All blocks run at the same 5 ms sample time.
%
%  --- PATH 1: Sensor-to-Host (Teensy sends measurements) ---
%
%    [MagSens] -----> (3*double: Bx,By,Bz)---\
%                                              \
%    [CurrSens 0] --> (single: I0) -----------\  \
%    [CurrSens 1] --> (single: I1) ----------\ \ |
%    [CurrSens 2] --> (single: I2) ---------\ \-+---> [UsbSerialPacketSend]
%    [CurrSens 3] --> (single: I3) --------\ \--+/
%                                           \---+/
%                                            \--/
%
%  --- PATH 2: Host-to-Actuator (Teensy receives commands) ---
%
%    [UsbSerialPacketReceive] --+--> (int16: cmd0) --> [Watchdog] --> [CurrDrv 0]
%                               +--> (int16: cmd1) --> [Watchdog] --> [CurrDrv 1]
%                               +--> (int16: cmd2) --> [Watchdog] --> [CurrDrv 2]
%                               +--> (int16: cmd3) --> [Watchdog] --> [CurrDrv 3]
%                               +--> (uint8: status) --> [Watchdog counter]
%
%  =========================================================================
%  STEP-BY-STEP INSTRUCTIONS
%  =========================================================================
%
%  1. CREATE NEW MODEL
%     - In MATLAB: Simulink > New > Blank Model
%     - Save as: Controller/hardware/TeensyNMPC_IO.slx
%     - Model Settings (Ctrl+E):
%         Solver > Type:            Fixed-step
%         Solver > Solver:          Discrete (no continuous states)
%         Solver > Fixed-step size: 0.005
%         Hardware Implementation > Hardware board: Teensy 4.1
%
%  2. ADD MAGNETIC SENSOR BLOCK
%     - From Simulink Library Browser: MagLev Toolbox > MagSens
%     - Mask parameters:
%         Sample time: 0.005
%     - (v4.3 has no sensorId parameter — single sensor)
%     - Output: double [3] = [Bx, By, Bz] in mT
%
%  3. ADD CURRENT SENSOR BLOCKS (x4)
%     - From Library Browser: MagLev Toolbox > CurrSens
%     - Create 4 instances with sensor numbers: 0, 1, 2, 3
%     - Each mask: Sample time = 0.005
%     - Output: single [1] = current in A
%
%  4. ADD USB SERIAL PACKET SEND BLOCK
%     - From Library Browser: MagLev Toolbox > UsbSerialPacketSend
%     - Mask parameters:
%         Packet specifier: {'3*double', 'single', 'single', 'single', 'single'}
%         COBS encoded:      on (checked)
%         Null terminated:   on (checked)
%         Wait for DTR:      on (checked)
%         Sample time:       0.005
%     - Connect inputs:
%         Port 1 <-- MagSens output   (3*double: Bx,By,Bz)
%         Port 2 <-- CurrSens 0       (single: I0)
%         Port 3 <-- CurrSens 1       (single: I1)
%         Port 4 <-- CurrSens 2       (single: I2)
%         Port 5 <-- CurrSens 3       (single: I3)
%
%  5. ADD USB SERIAL PACKET RECEIVE BLOCK
%     - From Library Browser: MagLev Toolbox > UsbSerialPacketReceive
%     - Mask parameters:
%         Packet specifier:   {'4*int16'}
%         COBS encoded:       on (checked)
%         Null terminated:    on (checked)
%         Show status port:   on (checked)
%         Output mode:        1  (hold last valid value)
%         Sample time:        0.005
%     - Outputs:
%         Port 1: int16 [4] = [cmd0, cmd1, cmd2, cmd3]
%         Port 2: uint8 [1] = status (1 = valid packet received)
%
%  6. ADD CURRENT DRIVER BLOCKS (x4)
%     - From Library Browser: MagLev Toolbox > CurrDrv
%     - Create 4 instances with driver numbers: 0, 1, 2, 3
%     - Each mask: Sample time = 0.005
%     - Input: int16 [1]
%     - Use Selector or Demux to split the 4*int16 vector from the
%       UsbSerialPacketReceive output into individual int16 scalars:
%         cmd(1) --> CurrDrv 0 (+X)
%         cmd(2) --> CurrDrv 1 (-X)
%         cmd(3) --> CurrDrv 2 (+Y)
%         cmd(4) --> CurrDrv 3 (-Y)
%
%  7. ADD SAFETY WATCHDOG
%     - Purpose: if no valid command packet arrives within 50 ms (10 steps),
%       override all CurrDrv inputs to zero.
%
%     - Implementation using standard Simulink blocks:
%
%       a) Watchdog counter subsystem:
%          - Input: status port (uint8) from UsbSerialPacketReceive
%          - Unit Delay (initial value 0, sample time 0.005)
%          - If status == 1: output 0 (reset counter)
%          - If status == 0: output = previous_count + 1
%          - Use a Switch block: condition (status > 0), true=0, false=(delay+1)
%
%       b) Command override:
%          - Compare counter output to threshold (10)
%          - If counter >= 10: output int16(0) to all CurrDrv blocks
%          - If counter < 10:  pass through the received commands
%          - Use a Switch block per driver:
%              condition (counter < 10)
%              true  = received command
%              false = int16(0) constant
%
%     - Alternatively, build this as a MATLAB Function block:
%
%         function [cmd_out, timeout] = watchdog(cmd_in, status)
%             persistent count
%             if isempty(count), count = int32(0); end
%
%             if status > 0
%                 count = int32(0);
%             else
%                 count = count + 1;
%             end
%
%             timeout = uint8(count >= 10);
%             if count >= 10
%                 cmd_out = int16([0; 0; 0; 0]);
%             else
%                 cmd_out = cmd_in;
%             end
%         end
%
%  8. BUILD AND DEPLOY
%     - Hardware tab > Build, Deploy & Start
%     - Wait for compilation and upload
%     - The model runs on the Teensy and begins sending sensor data
%     - Run hardwareNMPCController.m on the host to connect
%
%  =========================================================================
%  PACKET FORMAT SUMMARY
%  =========================================================================
%
%  Teensy --> Host (measurement packet):
%    packetSpec = {'3*double', 'single', 'single', 'single', 'single'}
%    Payload:     3*8 + 4*4 = 40 bytes
%    Content:     [Bx, By, Bz (mT), I0, I1, I2, I3 (A)]
%    Encoding:    COBS + null terminator
%
%  Host --> Teensy (command packet):
%    packetSpec = {'4*int16'}
%    Payload:     4*2 = 8 bytes
%    Content:     [cmd0, cmd1, cmd2, cmd3] in [-255, +255]
%    Encoding:    COBS + null terminator
%    Driver map:  cmd(1)=+X, cmd(2)=-X, cmd(3)=+Y, cmd(4)=-Y
%
%  =========================================================================
