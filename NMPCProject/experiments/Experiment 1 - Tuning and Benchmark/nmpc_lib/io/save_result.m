function path = save_result(rec, results_dir)
% SAVE_RESULT  Persist a result_record to disk.
%
%   path = save_result(rec, results_dir)
%
% File path:
%   <results_dir>/<stage>/<order>/<exp_id>__<cfg_hash[:8]>.mat
%
% Refuses to overwrite an existing file unless rec.cfg.allow_overwrite=true.
% On overwrite, prints a notice with the existing run_id so the user can
% reconcile.

    if nargin < 2 || isempty(results_dir)
        error('save_result:nargin','results_dir is required');
    end

    stage  = rec.cfg.stage;
    order  = rec.cfg.order;
    exp_id = rec.cfg.exp_id;

    dir_path = fullfile(results_dir, sanitize_(stage), sanitize_(order));
    if ~exist(dir_path,'dir'); mkdir(dir_path); end

    short_hash = rec.cfg_hash(1:8);
    fname = sprintf('%s__%s.mat', sanitize_(exp_id), short_hash);
    path  = fullfile(dir_path, fname);

    if exist(path, 'file')
        if isfield(rec.cfg,'allow_overwrite') && rec.cfg.allow_overwrite
            try
                existing = load(path, 'rec');
                if isfield(existing,'rec') && isfield(existing.rec,'run_id')
                    fprintf('[save_result] overwriting %s (existing run_id=%s)\n', ...
                            path, existing.rec.run_id);
                end
            catch
                fprintf('[save_result] overwriting %s (could not read existing)\n', path);
            end
        else
            error('save_result:exists', ...
                ['Result file already exists at %s. Set cfg.allow_overwrite=true ' ...
                 'to replace it.'], path);
        end
    end

    save(path, 'rec', '-v7.3'); %#ok<NASGU>
end

function s = sanitize_(s)
    s = regexprep(s, '[^A-Za-z0-9_\-]', '_');
end
