function emergencyStop(sp)
% EMERGENCYSTOP  Send zero-current command and close serial port.
%
%   emergencyStop(sp)
%
%   Sends a zero-current command to all 4 solenoid drivers via the serial
%   port, then closes the connection. Designed to be used with onCleanup
%   for safe shutdown on errors or Ctrl-C.

try
    serialPacketSend(sp, {int16([0; 0; 0; 0])}, true, true);
catch
end

try
    delete(sp);
catch
end

end
