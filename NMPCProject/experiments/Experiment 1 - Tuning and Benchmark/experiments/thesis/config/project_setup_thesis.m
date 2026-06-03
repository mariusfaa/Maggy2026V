function project_setup_thesis()
% PROJECT_SETUP_THESIS  Path + environment setup for the thesis framework.
%
% Adds the nmpc_lib subdirectories, the physics/utilities/parameter trees
% from this repo, and the acados MATLAB interfaces (if installed at the
% conventional location or pointed to via ACADOS_SOURCE_DIR).
%
% Idempotent: safe to call multiple times in a session.
%
% Notes
%   - Honors ACADOS_SOURCE_DIR / ACADOS_INSTALL_DIR / ENV_ACADOS_INSTALL_DIR
%     environment variables if set; falls back to "C:\Users\mariujf\acados"
%     (the path used by workingSimulator.m).
%   - Does NOT add the legacy nmpc_lib/ top-level directory to the path so
%     the new core/ and io/ implementations are never shadowed by the
%     legacy files (which use different cfg field names).

    % --- Repo root: this file lives at experiments/thesis/config/ ----
    here  = fileparts(mfilename('fullpath'));
    repo  = fileparts(fileparts(fileparts(here)));
    addpath(fullfile(repo, 'nmpc_lib', 'core'));
    addpath(fullfile(repo, 'nmpc_lib', 'metrics'));
    addpath(fullfile(repo, 'nmpc_lib', 'io'));
    addpath(fullfile(repo, 'nmpc_lib', 'plotting'));

    % --- Physics, utilities, parameters --------------------------------
    addpath(genpath(fullfile(repo, 'model_implementations')));
    addpath(genpath(fullfile(repo, 'system_parameters')));
    addpath(genpath(fullfile(repo, 'utilities')));

    % --- Thesis config / stages / analysis -----------------------------
    addpath(fullfile(repo, 'experiments', 'thesis'));                % top level (stageA_reset etc.)
    addpath(fullfile(repo, 'experiments', 'thesis', 'config'));
    addpath(fullfile(repo, 'experiments', 'thesis', 'stages'));
    addpath(fullfile(repo, 'experiments', 'thesis', 'analysis'));
    addpath(fullfile(repo, 'experiments', 'thesis', 'tests'));

    % --- acados --------------------------------------------------------
    acados_root = getenv('ACADOS_SOURCE_DIR');
    if isempty(acados_root); acados_root = getenv('ACADOS_INSTALL_DIR'); end
    if isempty(acados_root); acados_root = getenv('ENV_ACADOS_INSTALL_DIR'); end
    if isempty(acados_root) || ~exist(acados_root,'dir')
        % Convention used by workingSimulator.m.
        acados_root = 'C:\Users\mariujf\acados';
    end
    if exist(acados_root,'dir')
        setenv('ACADOS_SOURCE_DIR',      acados_root);
        setenv('ACADOS_INSTALL_DIR',     acados_root);
        setenv('ENV_ACADOS_INSTALL_DIR', acados_root);
        addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
        addpath(fullfile(acados_root, 'external',   'jsonlab'));
        addpath(fullfile(acados_root, 'external',   'casadi-matlab'));
    else
        warning('project_setup_thesis:no_acados', ...
                ['acados not found at %s and no ACADOS_SOURCE_DIR set. ' ...
                 'Set the env var or install acados; OCP/plant builds will fail.'], ...
                acados_root);
    end
end
