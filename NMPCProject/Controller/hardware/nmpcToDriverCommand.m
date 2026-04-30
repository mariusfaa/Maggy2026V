function cmd = nmpcToDriverCommand(u_nmpc)
% NMPCtodrivercommand  Convert NMPC solenoid currents to CurrDrv int16 commands.
%
%   cmd = nmpcToDriverCommand(u_nmpc)
%
%   Maps the NMPC output u_nmpc (4x1 double in [-1, 1]) to int16 commands
%   for the 4 CurrDrv blocks, handling both scaling and the permutation
%   between model solenoid order and hardware driver order.
%
%   Model solenoid order (parameters_maggy_V4.m):
%       u(1) = +X,  u(2) = +Y,  u(3) = -X,  u(4) = -Y
%
%   Hardware driver order (sfun_MagLevTbx_CurrDrv_WrappedFcns.cpp):
%       driver 0 = +X,  driver 1 = -X,  driver 2 = +Y,  driver 3 = -Y

%   Permute from model order [+X, +Y, -X, -Y] to driver order [+X, -X, +Y, -Y]
u_drivers = [u_nmpc(1); u_nmpc(3); u_nmpc(2); u_nmpc(4)];

%   Scale [-1, 1] to [-255, +255] (8-bit PWM resolution)
cmd = int16(round(u_drivers * 255));
cmd = max(min(cmd, int16(255)), int16(-255));

end
