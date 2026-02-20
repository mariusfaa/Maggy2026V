% MAGLEV_EXPORT_SCRIPT   Generate library Maglev.
% 
% Script generated from project 'maglev_export.prj' on 28-Dec-2025.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

%% Create configuration object of class 'coder.EmbeddedCodeConfig'.
cfg = coder.config('lib','ecoder',true);
cfg.HardwareImplementation.ProdHWDeviceType = 'ARM Compatible->ARM Cortex-M';
cfg.HardwareImplementation.TargetHWDeviceType = 'ARM Compatible->ARM Cortex-M';
cfg.TargetLang = 'C++';
cfg.CppNamespace = 'Maglev';

cfg.EnableDynamicMemoryAllocation = true;
cfg.EnableRuntimeRecursion = true;

cfg.GenerateDefaultInSwitch = true;
cfg.GenerateExampleMain = 'DoNotGenerate';
cfg.GenerateMakefile = false;
cfg.InlineBetweenUserAndMathWorksFunctions = 'Speed';
cfg.LargeConstantGeneration = 'KeepInSourceFiles';
cfg.MaxIdLength = 1024;
cfg.ReportPotentialDifferences = false;
cfg.RunInitializeFcn = false;
cfg.TargetLangStandard = 'C++17 (ISO)';
cfg.CodeReplacementLibrary = 'ARM Cortex-M';
cfg.Toolchain = 'GNU Tools for ARM Embedded Processors';
cfg.IncludeInitializeFcn = false;
cfg.IncludeTerminateFcn = false;

cfg.OptimizeReductions = true;
cfg.GenCodeOnly = true;

%% Invoke MATLAB Coder.
parameters_maggy_V4;
x0 = zeros(12,1);
u0 = zeros(length(params.solenoids.r),1);
xLp = x0;
uLp = u0;
k2 = 0.5;
k3 = 0.5*ones(1,21);
modelID = MaglevModel.Fast;

export_path = 'C:\Users\halva\Downloads\Maggy2026V\NMPCProject\MaglevLib';

codegen -config cfg -v -o MaglevLib -d 'C:\Users\halva\Downloads\Maggy2026V\NMPCProject\MaglevLib' ...
    maglevSystemDynamics -args {x0, u0, coder.Constant(params), coder.Constant(modelID)} ...
    ellipke -args {k2} ...
    ellipke -args {k3} ...
    maglevSystemMeasurements -args {x0, u0, coder.Constant(params), coder.Constant(modelID)} ...
    maglevSystemLinearized -args {xLp, uLp, coder.Constant(params), coder.Constant(modelID)}
% 
% packNGo(export_path, ...
%     'fileName', 'myAdd_codegen', ...
%     'packType', 'hierarchical');

%% Remove junk
% From export_path remove relative dirs "./examples", "./interface",
% keep only *.c*, *.h* files, remove rest

disp("Cleaning up...")

% Folders to remove explicitly
junkDirs = {'examples', 'interface'};

for k = 1:numel(junkDirs)
    d = fullfile(export_path, junkDirs{k});
    if exist(d, 'dir')
        rmdir(d, 's');
    end
end

% Allowed file extensions
keepExt = {'.c', '.cpp', '.h', '.hpp'};

% List all items in export_path
items = dir(export_path);

for k = 1:numel(items)
    name = items(k).name;

    % Skip current/parent directory
    if strcmp(name, '.') || strcmp(name, '..')
        continue
    end

    fullItemPath = fullfile(export_path, name);

    if items(k).isdir
        % Remove any remaining directories
        rmdir(fullItemPath, 's');
    else
        % Check file extension
        [~, ~, ext] = fileparts(name);
        if ~ismember(lower(ext), keepExt)
            delete(fullItemPath);
        end
    end
end

disp("Done!")