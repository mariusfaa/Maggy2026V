function [brho,bz] = computeFieldCircularCurrentSheetPolarSmart(rho,z,r,l,mu0)
% COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR computes the magnetic field in 
% polar coordinates produced by a cylindrical current carrying sheet 
% (an ideal solenoid). The current sheet is centered at the origin, with 
% its main axis aligned with the z-axis.
%
% The function calculates the magnetic field components (bphi, brho, bz) at
% the specified points (phi, rho, z) in polar coordinates. The strength and
% direction of the magnetic field is determined by the radius r of current 
% sheet, the current I running through it and the magnetic permeability of 
% the medium (e.g., air).
%
% Example:
%   phi = [0, pi/4, pi/2]; rho = [1, 1.5, 2]; z = [0, 0.5, 1];
%   r = 1; I = 1; mu0 = 4*pi*1e-7;
%   [bphi, brho, bz] = computeFieldCircularCurrentSheetPolar(phi,rho,z,r,I,mu0);
%
% See also COMPUTEFIELDCIRCULARCURRENTSHEETCARTESIAN, 
%          COMPUTEFIELDCIRCULARWIREPOLAR.

% COMPUTEFIELDCIRCULARCURRENTSHEETPOLAR uses the implementation described in [1].

% Author: Hans Alvar Engmark
% Date: 08.01.2024

% References:
% [1] Engmark, Hans Alvar, and Kiet Tuan Hoang. 
%     "Modeling and Control of a Magnetic Levitation Platform." 
%     IFAC-PapersOnLine 56.2 (2023): 7276-7281.
%% rho ~= 0 (2a)
tol_r = 1e-9;                  % very small offset
rho(abs(rho-r) < tol_r) = r + tol_r;

h2 = 4*r*rho./(r+rho).^2;
c = mu0./(4*pi*sqrt(r.*rho))/l;

zetap = z + l/2;
zetan = z - l/2;

k2p = 4*r*rho./((r+rho).^2+zetap.^2);
k2n = 4*r*rho./((r+rho).^2+zetan.^2);

if ~isa(k2p, 'sym')
    eps_k = 1e-9;
    k2p = min(max(k2p,0),1-eps_k);
    k2n = min(max(k2n,0),1-eps_k);
end

tol = 1e-6;
[Kp,Ep] = ellipke(k2p,tol);
[Kn,En] = ellipke(k2n,tol);

Pp = ellipP(h2, k2p);
Pn = ellipP(h2, k2n);

brhop = c.*2.*r.*(k2p-2)./(sqrt(k2p)).*(Kp-2./(2-k2p).*Ep);
brhon = c.*2.*r.*(k2n-2)./(sqrt(k2n)).*(Kn-2./(2-k2n).*En);
brho = brhop - brhon;

% bzp   = c.*zetap.*sqrt(k2p).*(Kp - ((rho-r)./(rho+r)).*Pp);
% bzn   = c.*zetan.*sqrt(k2n).*(Kn - ((rho-r)./(rho+r)).*Pn);
% bz = bzp - bzn;

% bz
alpha = (rho - r) ./ (rho + r);
alpha(abs(alpha) < 1e-12) = 0;  % stabilize exactly at rho=r

bzp = c .* zetap .* sqrt(k2p) .* (Kp - alpha .* Pp);
bzn = c .* zetan .* sqrt(k2n) .* (Kn - alpha .* Pn);
bz = bzp - bzn;

%% rho = 0 (2b)
tol = 1e-6;
if ~isa(rho, 'sym')
    indices = abs(rho) < tol;
    brho(indices) = 0;
    bz(indices)   = mu0*zetap(indices)./(2*l*sqrt(r^2+zetap(indices).^2)) - ...
                    mu0*zetan(indices)./(2*l*sqrt(r^2+zetan(indices).^2));
end