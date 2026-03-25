% Script loads all paths that are needed for the project to run
% It assumes the user has set the following enviromental paths
% - ACADOS_INSTALL_DIR : Points to the acados root directory
% - ACADOS_PROJECT_DIR : Points to the root directory of this project
% NOTE: see env.sh to easily edit these paths

acados_root  = getenv('ACADOS_INSTALL_DIR');
project_root = getenv('ACADOS_PROJECT_DIR');

assert(~isempty(acados_root),  'ACADOS_INSTALL_DIR not set. Source env.sh first.');
assert(~isempty(project_root), 'ACADOS_PROJECT_DIR not set. Source env.sh first.');

addpath(fullfile(acados_root, 'interfaces', 'acados_matlab_octave'));
addpath(fullfile(acados_root, 'external',   'jsonlab'));
addpath(fullfile(acados_root, 'external',   'casadi-matlab'));

addpath(genpath(fullfile(project_root, 'model_matlab')));
% addpath(genpath(fullfile(project_root, 'model_casadi')));
addpath(genpath(fullfile(project_root, 'model_reduced_casadi')));
% addpath(genpath(fullfile(project_root, 'model_dipole_casadi')));
addpath(genpath(fullfile(project_root, 'system_parameters')));
addpath(genpath(fullfile(project_root, 'utilities')));

setenv('GCC','ccache gcc');
setenv('CC','ccache gcc');
setenv('CXX','ccache g++');