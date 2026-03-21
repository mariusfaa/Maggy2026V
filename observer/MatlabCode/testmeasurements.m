
addpath(genpath('~/MSc/Take-home-Maglev-lab_mod'));

x = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
u = [1,0,-1,0]';

matlabres = maglevSystemMeasurements_accurate(x,u);

mexres = maglevSystemMeasurements_accurate_mex(x,u);

norm(matlabres - mexres)