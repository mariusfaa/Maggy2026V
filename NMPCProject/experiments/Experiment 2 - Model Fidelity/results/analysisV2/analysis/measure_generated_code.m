function T = measure_generated_code(T, code_root)
% MEASURE_GENERATED_CODE  Augment sweep table with per-run code-size metrics.
%
%   T = measure_generated_code(T, code_root) walks the archived code tree at
%   code_root (e.g. results_magnet_n_sweep/generated_code) and, for each row
%   of T with a matching run_id, computes:
%       per_run_bytes              total size of files whose name contains run_id
%       per_run_c_lines            total lines in *.c files matching run_id
%       per_run_dyn_c_lines        total lines in dynamics-related *.c files
%                                   (impl_dae, expl_ode, cost, constraints)
%
%   The CSV's generated_code_bytes column reflects the cumulative state of
%   the working c_generated_code/ directory at archive time and therefore
%   overcounts later runs. The per_run_* columns isolate the contribution
%   actually attributable to that specific run.

    if nargin < 2 || isempty(code_root)
        here = fileparts(mfilename('fullpath'));
        code_root = fullfile(here, '..', 'generated_code');
    end
    code_root = char(code_root);

    n = height(T);
    per_run_bytes       = zeros(n, 1);
    per_run_c_lines     = zeros(n, 1);
    per_run_dyn_c_lines = zeros(n, 1);

    for i = 1:n
        rid_raw = T.run_id(i);
        if ismissing(rid_raw) || strlength(string(rid_raw)) == 0
            continue;
        end
        run_id = char(string(rid_raw));
        run_dir = fullfile(code_root, run_id);
        if ~isfolder(run_dir)
            continue;
        end

        all_entries = dir(fullfile(run_dir, '**', '*'));
        all_entries = all_entries(~[all_entries.isdir]);

        names = string({all_entries.name});
        keep = contains(names, run_id);
        if ~any(keep)
            continue;
        end

        sub = all_entries(keep);
        per_run_bytes(i) = sum([sub.bytes]);

        is_c = endsWith(string({sub.name}), '.c');
        c_files = sub(is_c);

        if ~isempty(c_files)
            per_run_c_lines(i)     = sum_lines(c_files);
            dyn_mask = matches_dyn(string({c_files.name}));
            per_run_dyn_c_lines(i) = sum_lines(c_files(dyn_mask));
        end
    end

    T.per_run_bytes       = per_run_bytes;
    T.per_run_c_lines     = per_run_c_lines;
    T.per_run_dyn_c_lines = per_run_dyn_c_lines;
end

function total = sum_lines(file_structs)
    total = 0;
    for k = 1:numel(file_structs)
        fp = fullfile(file_structs(k).folder, file_structs(k).name);
        fid = fopen(fp, 'r');
        if fid < 0
            continue;
        end
        cleanup = onCleanup(@() fclose(fid));
        nl = 0;
        chunk_size = 1048576;
        while ~feof(fid)
            buf = fread(fid, chunk_size, 'uint8=>uint8');
            if isempty(buf)
                break;
            end
            nl = nl + sum(buf == 10);
        end
        total = total + nl;
        clear cleanup;
    end
end

function tf = matches_dyn(names)
    patterns = {'_impl_dae_', '_expl_ode_', '_cost', '_constraints', '_model'};
    tf = false(size(names));
    for p = 1:numel(patterns)
        tf = tf | contains(names, patterns{p});
    end
    excl = {'acados_solver_', 'acados_sim_solver_', 'acados_mex_', ...
            'acados_sim_create_', 'acados_sim_set_', 'acados_sim_free_', ...
            'sfunction_'};
    for p = 1:numel(excl)
        tf = tf & ~contains(names, excl{p});
    end
end
