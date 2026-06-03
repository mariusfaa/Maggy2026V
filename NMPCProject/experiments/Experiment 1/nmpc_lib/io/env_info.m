function info = env_info()
% ENV_INFO  Collect environment metadata for embedding in result records.
%
%   info = env_info()
%
% Returns a struct with the following fields (all char):
%   .timestamp           ISO 8601 local time
%   .schema_version      Result-record schema version (declared here)
%   .matlab_version      e.g. "9.14.0.2206163 (R2023a)"
%   .casadi_version      from casadi.CasadiMeta.version() or "unknown"
%   .acados_install_dir  resolved from ACADOS_SOURCE_DIR / ACADOS_INSTALL_DIR / "unknown"
%   .acados_version      best-effort parse of <acados>/CHANGELOG.md or <acados>/.git HEAD, or "unknown"
%   .repo_root           inferred experiment-repo root (containing workingSimulator.m), or "unknown"
%   .git_commit          repo HEAD (short hash) if a .git is present, else "NA"
%   .host                computer name (COMPUTERNAME on Windows, HOSTNAME elsewhere)
%   .os                  e.g. "Windows 11 Pro 10.0.26200" or "Linux x.y.z"
%   .cpu                 best-effort CPU model string, or "unknown"
%
% This function is robust: every probe is try/catched and falls back to a
% sentinel value rather than throwing. It is safe to call from any stage.

    info.timestamp      = local_timestamp();
    info.schema_version = '2026.05.13.1';

    % --- MATLAB ----------------------------------------------------------
    try
        info.matlab_version = version();
    catch
        info.matlab_version = 'unknown';
    end

    % --- CasADi ----------------------------------------------------------
    info.casadi_version = 'unknown';
    try
        % Lazy import: only touch casadi if it is on the path.
        if exist('casadi.CasadiMeta','class') == 8
            v = casadi.CasadiMeta.version();
            if ischar(v); info.casadi_version = v;
            else;          info.casadi_version = char(v); end
        end
    catch
        % keep 'unknown'
    end

    % --- acados ----------------------------------------------------------
    [info.acados_install_dir, info.acados_version] = probe_acados();

    % --- experiment repo + git -------------------------------------------
    [info.repo_root, info.git_commit] = probe_repo();

    % --- host / OS -------------------------------------------------------
    info.host = host_name();
    info.os   = os_string();
    info.cpu  = cpu_model();
end

function ts = local_timestamp()
    try
        ts = char(datetime('now','Format','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','local'));
    catch
        % datetime missing on very old MATLAB.
        ts = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
    end
end

function [install_dir, version_str] = probe_acados()
    install_dir = 'unknown';
    version_str = 'unknown';
    candidates = {getenv('ACADOS_SOURCE_DIR'), getenv('ACADOS_INSTALL_DIR'), ...
                  getenv('ENV_ACADOS_INSTALL_DIR')};
    for k = 1:numel(candidates)
        c = candidates{k};
        if ~isempty(c) && exist(c,'dir')
            install_dir = c; break;
        end
    end
    if strcmp(install_dir,'unknown'); return; end

    % Try version.txt (some acados builds ship one).
    cand_files = {fullfile(install_dir,'version.txt'), ...
                  fullfile(install_dir,'VERSION'), ...
                  fullfile(install_dir,'CHANGELOG.md'), ...
                  fullfile(install_dir,'CHANGELOG')};
    for k = 1:numel(cand_files)
        if exist(cand_files{k},'file')
            try
                fid = fopen(cand_files{k},'rt');
                if fid > 0
                    line = fgetl(fid);
                    fclose(fid);
                    if ischar(line)
                        version_str = strtrim(line);
                        % CHANGELOG.md typically starts with "# Changelog"; try harder.
                        if startsWith(version_str,'#')
                            v = parse_first_version(cand_files{k});
                            if ~isempty(v); version_str = v; end
                        end
                        return
                    end
                end
            catch
                % fall through to next candidate
            end
        end
    end

    % Fall back to git HEAD if acados is itself a git checkout.
    try
        head = fullfile(install_dir, '.git', 'HEAD');
        if exist(head,'file')
            fid = fopen(head,'rt'); line = fgetl(fid); fclose(fid);
            if ischar(line)
                if startsWith(line,'ref: ')
                    ref_path = fullfile(install_dir, '.git', strtrim(line(6:end)));
                    if exist(ref_path,'file')
                        fid = fopen(ref_path,'rt'); sha = fgetl(fid); fclose(fid);
                        version_str = ['git:' strtrim(sha(1:min(end,12)))];
                    end
                else
                    version_str = ['git:' strtrim(line(1:min(end,12)))];
                end
            end
        end
    catch
        % keep 'unknown'
    end
end

function v = parse_first_version(path)
    v = '';
    try
        txt = fileread(path);
        m = regexp(txt, '\d+\.\d+(\.\d+)?', 'match', 'once');
        if ~isempty(m); v = m; end
    catch
    end
end

function [repo_root, git_commit] = probe_repo()
    repo_root = 'unknown';
    git_commit = 'NA';

    % Search upward from this file's dir until a marker is found.
    here = fileparts(mfilename('fullpath'));
    here = fileparts(here);  % up from io/ to nmpc_lib/
    here = fileparts(here);  % up from nmpc_lib/ to repo root
    if exist(fullfile(here,'workingSimulator.m'),'file')
        repo_root = here;
    end

    if ~strcmp(repo_root,'unknown')
        gitdir = fullfile(repo_root, '.git');
        if exist(gitdir,'dir') || exist(gitdir,'file')
            try
                head = fullfile(gitdir,'HEAD');
                if exist(head,'file')
                    fid = fopen(head,'rt'); line = fgetl(fid); fclose(fid);
                    if ischar(line) && startsWith(line,'ref: ')
                        ref_path = fullfile(gitdir, strtrim(line(6:end)));
                        if exist(ref_path,'file')
                            fid = fopen(ref_path,'rt'); sha = fgetl(fid); fclose(fid);
                            git_commit = strtrim(sha(1:min(end,12)));
                        end
                    elseif ischar(line)
                        git_commit = strtrim(line(1:min(end,12)));
                    end
                end
            catch
                git_commit = 'NA';
            end
        end
    end
end

function h = host_name()
    h = getenv('COMPUTERNAME');
    if isempty(h); h = getenv('HOSTNAME'); end
    if isempty(h)
        try
            [s, out] = system('hostname');
            if s == 0; h = strtrim(out); end
        catch
        end
    end
    if isempty(h); h = 'unknown'; end
end

function s = os_string()
    s = 'unknown';
    if ispc
        try
            [st, out] = system('ver');
            if st == 0; s = strtrim(out); end
        catch
        end
    elseif isunix
        try
            [st, out] = system('uname -srm');
            if st == 0; s = strtrim(out); end
        catch
        end
    end
end

function s = cpu_model()
    s = 'unknown';
    try
        if ispc
            [st, out] = system('wmic cpu get Name /value');
            if st == 0
                m = regexp(out, 'Name=([^\r\n]+)', 'tokens', 'once');
                if ~isempty(m); s = strtrim(m{1}); end
            end
        elseif ismac
            [st, out] = system('sysctl -n machdep.cpu.brand_string');
            if st == 0; s = strtrim(out); end
        elseif isunix
            [st, out] = system('grep -m1 "model name" /proc/cpuinfo | cut -d: -f2');
            if st == 0; s = strtrim(out); end
        end
    catch
    end
end
