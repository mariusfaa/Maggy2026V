%% compile_observer.m
% Script to compile the MEX wrapper

% Add paths to necessary libraries
armadillo_path = '/usr/include/armadillo';
observer_path = '~/MSc/Maggy2026V/observer/PCobserver/';

% Source files
cpp_files = {
    [observer_path 'observer.cpp'], ...
    [observer_path 'matrices.cpp'], ...
    [observer_path 'utilities.cpp'], ...
    [observer_path 'observer_mex.cpp']
};

% Include paths
include_paths = {
    ['-I' observer_path 'include'], ...
    ['-I' '/usr/include']
};

% Library paths and flags
lib_flags = {
    ['-L' observer_path 'lib'], ...
    ['-L' '/usr/lib'], ...
    ['-l' 'armadillo'], ...
    ['-l' 'maglevModel']
    %['-f' 'openmp']
};

% Compilation command
mex_command = ['mex ' strjoin(include_paths) ' ' ...
    strjoin(cpp_files) ' ' strjoin(lib_flags)];

% Execute compilation
disp('Compiling observer MEX...');
eval(mex_command)
disp('Compilation complete!');