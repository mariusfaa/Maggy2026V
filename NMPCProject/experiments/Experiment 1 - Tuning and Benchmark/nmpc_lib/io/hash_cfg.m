function hex = hash_cfg(cfg)
% HASH_CFG  Deterministic SHA-256 of a canonicalised cfg struct.
%
%   hex = hash_cfg(cfg)
%
% Two cfgs that are semantically identical produce the same hash, regardless
% of field order, numeric formatting, or whitespace. The canonical form is:
%   - struct: sorted field names, each rendered as "name:value"
%   - numeric vector/matrix: row-major, %.17g per element
%   - char:    quoted string with double quotes around the content
%   - logical: "true" or "false"
%   - cell:    bracketed, recursively rendered
%   - empty:   "[]"
%   - function_handle: func2str()
%
% The resulting canonical string is SHA-256-hashed via java.security.MessageDigest
% (always available in MATLAB) and returned as a lowercase hex string.
%
% Use this for cfg_hash on result records and as the key for file caches.

    s   = canonical(cfg);
    md  = java.security.MessageDigest.getInstance('SHA-256');
    h   = md.digest(uint8(s));
    hex = lower(reshape(dec2hex(typecast(h,'uint8'), 2).', 1, []));
end

function s = canonical(v)
    if isstruct(v)
        if isempty(v)
            s = 'struct()';
            return
        end
        fns = sort(fieldnames(v));
        if numel(v) > 1
            parts = cell(numel(v),1);
            for i = 1:numel(v)
                inner = cell(numel(fns),1);
                for j = 1:numel(fns)
                    inner{j} = [fns{j} ':' canonical(v(i).(fns{j}))];
                end
                parts{i} = ['{' strjoin(inner, ';') '}'];
            end
            s = ['[' strjoin(parts, ',') ']'];
        else
            inner = cell(numel(fns),1);
            for j = 1:numel(fns)
                inner{j} = [fns{j} ':' canonical(v.(fns{j}))];
            end
            s = ['{' strjoin(inner, ';') '}'];
        end
    elseif isnumeric(v)
        if isempty(v)
            s = '[]';
        elseif isscalar(v)
            s = sprintf('%.17g', double(v));
        else
            n  = numel(v);
            vv = double(v(:));
            parts = cell(n,1);
            for k = 1:n
                parts{k} = sprintf('%.17g', vv(k));
            end
            sz = sprintf('%dx%d', size(v,1), size(v,2));
            s  = ['n[' sz '|' strjoin(parts, ',') ']'];
        end
    elseif ischar(v)
        s = ['"' v '"'];
    elseif islogical(v)
        if isscalar(v)
            if v; s = 'true'; else; s = 'false'; end
        else
            vv = double(v(:));
            parts = cell(numel(vv),1);
            for k = 1:numel(vv); if vv(k); parts{k}='1'; else; parts{k}='0'; end; end
            s = ['b[' strjoin(parts, ',') ']'];
        end
    elseif iscell(v)
        parts = cell(numel(v),1);
        for k = 1:numel(v)
            parts{k} = canonical(v{k});
        end
        s = ['c[' strjoin(parts, ',') ']'];
    elseif isa(v, 'function_handle')
        s = ['fh<' func2str(v) '>'];
    elseif isstring(v)
        % MATLAB string type (R2016b+). Convert to char for canonical form.
        if isscalar(v)
            s = ['"' char(v) '"'];
        else
            parts = cell(numel(v),1);
            for k = 1:numel(v); parts{k} = ['"' char(v(k)) '"']; end
            s = ['s[' strjoin(parts, ',') ']'];
        end
    else
        % Fallback: class name + serialized bytes. Not great, but deterministic.
        s = ['?<' class(v) '>'];
    end
end
