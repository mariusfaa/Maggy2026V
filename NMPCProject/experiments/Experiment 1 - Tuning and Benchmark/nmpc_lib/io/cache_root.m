function root = cache_root(sub)
% CACHE_ROOT  Resolve and (on first use) create a persistent cache directory.
%
%   root = cache_root()           -> ~/.cache/maglev_thesis/
%   root = cache_root('dynamics') -> ~/.cache/maglev_thesis/dynamics/
%
% Uses USERPROFILE on Windows, HOME on Unix, with java.lang.System.getProperty
% as a final fallback. The directory is created if it does not exist.

    if nargin < 1; sub = ''; end

    home = getenv('USERPROFILE');
    if isempty(home); home = getenv('HOME'); end
    if isempty(home)
        try
            home = char(java.lang.System.getProperty('user.home'));
        catch
            home = pwd;
        end
    end

    root = fullfile(home, '.cache', 'maglev_thesis');
    if ~isempty(sub); root = fullfile(root, sub); end
    if ~exist(root, 'dir'); mkdir(root); end
end
