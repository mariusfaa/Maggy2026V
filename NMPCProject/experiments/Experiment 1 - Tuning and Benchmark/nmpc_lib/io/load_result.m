function rec = load_result(path)
% LOAD_RESULT  Load a result_record from disk and verify its schema.
%
%   rec = load_result(path)
%
% Throws if the file does not exist or if rec.schema_version differs from
% the current framework schema_version (a downstream script must opt in
% to back-compat reads explicitly via load() instead).

    if ~exist(path,'file')
        error('load_result:no_file','No result at %s', path);
    end
    S = load(path, 'rec');
    if ~isfield(S,'rec')
        error('load_result:malformed','Expected variable "rec" in %s', path);
    end
    rec = S.rec;

    current_info    = env_info();
    current_version = current_info.schema_version;
    if ~isfield(rec,'schema_version') || ~strcmp(rec.schema_version, current_version)
        existing = ifelse_(isfield(rec,'schema_version'), rec.schema_version, '<missing>');
        error('load_result:schema_mismatch', ...
              'Result schema_version=%s but framework expects %s. Use load() directly to bypass.', ...
              existing, current_version);
    end
end

function y = ifelse_(c, a, b)
    if c; y = a; else; y = b; end
end
