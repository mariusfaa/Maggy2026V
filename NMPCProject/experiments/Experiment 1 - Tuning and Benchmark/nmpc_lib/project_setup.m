function paths = project_setup()
% PROJECT_SETUP  Initialize MATLAB paths and acados env vars. Self-locates
% via mfilename so works regardless of the caller's CWD.
%
%   paths = project_setup()
%
% Returns
%   paths.acados_root      absolute path to acados install
%   paths.project_root     absolute path to NMPCProject
%   paths.lib_dir          absolute path to NMPCProject/nmpc_lib
%   paths.experiments_dir  absolute path to NMPCProject/experiments
%
% This function lives in nmpc_lib/, so:
%   mfilename('fullpath') -> .../NMPCProject/nmpc_lib/project_setup
% from which we derive project_root by going up one directory.

    this_file = mfilename('fullpath');
    [lib_dir, ~]      = fileparts(this_file);                 % .../nmpc_lib
    [project_root, ~] = fileparts(lib_dir);                   % .../NMPCProject

    paths.lib_dir         = lib_dir;
    paths.project_root    = project_root;
    paths.experiments_dir = fullfile(project_root, 'experiments');

    % --- Enter the path to your local acados installation here ---
    if ispc
        paths.acados_root = '';   % e.g. 'C:\Users\you\acados'
    else
        paths.acados_root = '';   % e.g. '/home/you/acados'
    end

    setenv('ACADOS_SOURCE_DIR',      paths.acados_root);
    setenv('ENV_ACADOS_INSTALL_DIR', paths.acados_root);
    setenv('ACADOS_INSTALL_DIR',     paths.acados_root);

    addpath(fullfile(paths.acados_root, 'interfaces', 'acados_matlab_octave'));
    addpath(fullfile(paths.acados_root, 'external',   'jsonlab'));
    addpath(fullfile(paths.acados_root, 'external',   'casadi-matlab'));

    addpath(genpath(fullfile(paths.project_root, 'model_implementations')));
    addpath(genpath(fullfile(paths.project_root, 'system_parameters')));
    addpath(genpath(fullfile(paths.project_root, 'utilities')));
    addpath(paths.lib_dir);
end
