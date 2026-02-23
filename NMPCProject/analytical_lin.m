
acados_root = 'C:\Users\halva\Downloads\acados';
project_root = 'C:\Users\halva\Downloads\Maggy2026V\NMPCProject';

% Add project folders to path
addpath(genpath(fullfile(project_root, 'model_implementations')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

load_params;

x_sym = sym('x', [12 1], 'real');
u_sym = sym('u', [4 1], 'real');

dx_sym = maglevSystemDynamicsSym(x_sym, u_sym, params, modelId);

%save("symdata.mat", "dx_sym", "params", "modelId")