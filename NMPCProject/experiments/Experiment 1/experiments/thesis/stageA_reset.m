% STAGEA_RESET  Clean slate for the Stage A validation gate.
%
% Run this BEFORE stageA_validate when you want a from-scratch invocation.
% It:
%   1. Releases MATLAB's hold on previously loaded acados MEX classes /
%      DLLs (so re-building the same model_name doesn't fail with
%      "Permission denied" at the linker on Windows).
%   2. Deletes the Stage A results tree (so stageA_validate can write
%      fresh files without needing allow_overwrite).
%   3. Deletes the acados c_generated_code / c_generated_code_plant
%      directories under the repo (forces a clean rebuild; otherwise
%      stale binaries from past test runs can shadow new builds).
%
% Use as a SCRIPT from MATLAB's base workspace:
%
%     stageA_reset;
%     report = stageA_validate;
%
% This is a script (not a function) on purpose: 'clear classes' only
% releases handles in the workspace it runs in.
%
% File caches under ~/.cache/maglev_thesis/ are NOT cleared by this
% script -- the correction-factor fmincon and the equilibrium IPOPT
% are expensive to redo and their results depend only on (magnet_n,
% params_hash), which haven't changed. Delete that directory by hand
% if you really want a from-scratch params recomputation.

fprintf('[stageA_reset] releasing MATLAB-side MEX handles...\n');
clear classes %#ok<CLCLS>
clear functions %#ok<CLFUNC>
clear mex
clear variables

% --- Locate repo root from this file's path -------------------------
ws_here   = which('stageA_reset');
ws_dir    = fileparts(ws_here);
ws_repo   = fileparts(fileparts(ws_dir));   % up two from experiments/thesis/
fprintf('[stageA_reset] repo: %s\n', ws_repo);

% --- 1. Results tree ------------------------------------------------
ws_results = fullfile(ws_repo, 'experiments', 'thesis', 'results', 'stageA');
if exist(ws_results, 'dir')
    fprintf('[stageA_reset] deleting %s ...\n', ws_results);
    rmdir(ws_results, 's');
    fprintf('[stageA_reset]   gone.\n');
else
    fprintf('[stageA_reset] %s does not exist; skipped.\n', ws_results);
end

% --- 2. Codegen dirs under the repo ---------------------------------
% Acados writes c_generated_code/ and c_generated_code_plant/ relative
% to pwd at build time. Past test/Stage-A runs may have left them in
% several places; remove every match under the repo.
ws_candidates = { ...
    fullfile(ws_repo, 'c_generated_code'), ...
    fullfile(ws_repo, 'c_generated_code_plant'), ...
    fullfile(ws_repo, 'experiments', 'thesis', 'c_generated_code'), ...
    fullfile(ws_repo, 'experiments', 'thesis', 'c_generated_code_plant'), ...
    fullfile(ws_repo, 'experiments', 'thesis', 'tests', 'c_generated_code'), ...
    fullfile(ws_repo, 'experiments', 'thesis', 'tests', 'c_generated_code_plant'), ...
    fullfile(ws_repo, 'experiments', 'thesis', 'stages', 'c_generated_code'), ...
    fullfile(ws_repo, 'experiments', 'thesis', 'stages', 'c_generated_code_plant') ...
};
for ws_k = 1:numel(ws_candidates)
    ws_p = ws_candidates{ws_k};
    if exist(ws_p, 'dir')
        fprintf('[stageA_reset] deleting %s ...\n', ws_p);
        try
            rmdir(ws_p, 's');
            fprintf('[stageA_reset]   gone.\n');
        catch ws_err
            fprintf('[stageA_reset]   FAILED: %s (a DLL may still be loaded; restart MATLAB and retry)\n', ws_err.message);
        end
    end
end

clear ws_here ws_dir ws_repo ws_results ws_candidates ws_k ws_p ws_err
fprintf('[stageA_reset] done. Now run:  report = stageA_validate;\n');
