function paths = project_setup()
% PROJECT_SETUP  Initialize MATLAB paths and acados env vars for the
% benchmark. Idempotent — safe to call multiple times.
%
%   paths = project_setup()
%
% Returns
%   paths.acados_root   absolute path to acados install
%   paths.project_root  absolute path to NMPCProject

    if ispc
        paths.acados_root  = 'C:\Users\mariujf\acados';
        paths.project_root = 'C:\Users\mariujf\Maggy2026V\NMPCProject';
    else
        paths.acados_root  = '/home/mariujf/acados';
        paths.project_root = '/home/mariujf/Maggy2026V/NMPCProject';
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
end
