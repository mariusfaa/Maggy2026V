
addpath(genpath('~/MSc/Take-home-Maglev-lab_mod'));
addpath(genpath('~/MSc/Cmodel'));


x = [0,0,0.1,0,0,0,0,0,0,0,0,0]';
u = [1,0,-1,0]';

matlabres = maglevSystemDynamics(x,u)
matlabres_accurate = maglevSystemDynamics_accurate(x,u)

mexres = maglevSystemDynamics_mex(x,u);
mexres_accurate = maglevSystemDynamics_mex(x,u);

norm(matlabres - matlabres_accurate)