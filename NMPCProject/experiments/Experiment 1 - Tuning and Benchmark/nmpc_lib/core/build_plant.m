function plant_solver = build_plant(M, cfg)
% BUILD_PLANT  Build an acados sim solver for the 12-state plant using the
% matched-plant policy.
%
%   plant_solver = build_plant(M, cfg)
%
% Inputs
%   M      struct from build_model(). Uses M.x_plant_sym, M.xdot_plant_sym,
%          M.u_sym, M.f_expl_plant.
%   cfg    validated cfg (use validate_cfg first). The plant integrator
%          settings derive from cfg's OCP integrator fields by the
%          matched-plant policy:
%             integrator_type = cfg.integrator_type
%             num_stages      = cfg.sim_method_num_stages
%             num_steps       = cfg.sim_method_num_steps
%             Tsim            = cfg.Tf / cfg.N
%          Any non-empty cfg.plant_override.* OVERRIDES the corresponding
%          field, marking the build as asymmetric.
%
% Output
%   plant_solver  AcadosSimSolver with the resolved settings.
%
% Caching
%   Session-persistent in-memory cache keyed on:
%       (M.order, M.magnet_n_plant, integrator_type, num_stages,
%        num_steps, round(Tsim*1e9), params_hash)
%   so distinct configurations get distinct cached solvers.
%
% Acados c_generated_code is written to <pwd>/c_generated_code_plant/ to
% avoid the OCP solver's post-build cleanup invalidating the cached plant
% solver class file.

    if nargin < 2; error('build_plant:nargin', 'Usage: build_plant(M, cfg)'); end

    % --- Resolve matched-plant + override settings -----------------------
    [itype, nstages, nsteps] = resolve_plant_integrator(cfg);
    Tsim = cfg.Tf / cfg.N;

    % --- Cache key ------------------------------------------------------
    % acados' MATLAB binding zombifies previously-cached AcadosSimSolver MEX
    % classes whenever a new AcadosOcpSolver is constructed in the same
    % session (the .solve() handle starts returning NaN even though x and u
    % are finite). So the cache key must include the *full* cfg identity,
    % not just the plant settings -- and the rest of the framework must
    % rebuild the plant after every OCP build (Stage B's run_one_config_
    % already does this).
    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType','char','ValueType','any');
    end
    cfg_id = '';
    if isfield(cfg,'exp_id') && ischar(cfg.exp_id); cfg_id = cfg.exp_id; end
    cfg_hash_short = hash_cfg(cfg); cfg_hash_short = cfg_hash_short(1:8);
    key = sprintf('%s_n%d_%s_s%d_t%d_T%d_%s_%s_%s', ...
        M.order, M.magnet_n_plant, itype, nstages, nsteps, ...
        round(Tsim*1e9), M.equilibrium.params_hash(1:8), cfg_id, cfg_hash_short);
    if isKey(cache, key)
        plant_solver = cache(key);
        return
    end

    % --- Build -----------------------------------------------------------
    sim = AcadosSim();
    % Unique model name per cfg_hash so the generated DLL filename is unique
    % too -- otherwise on Windows the linker fails with "Permission denied"
    % when trying to overwrite a DLL that's still loaded in this session.
    % Keep it SHORT (Windows MAX_PATH = 260) -- CMake builds intermediate
    % paths of the form CMakeFiles\model_<name>.dir\<subhash>\<name>_impl_dae_...
    sim.model.name        = sprintf('p_%s', cfg_hash_short);
    sim.model.x           = M.x_plant_sym;
    sim.model.u           = M.u_sym;
    sim.model.xdot        = M.xdot_plant_sym;
    sim.model.f_impl_expr = M.xdot_plant_sym - M.f_expl_plant;
    if strcmp(itype,'ERK')
        % ERK uses explicit form.
        sim.model.f_expl_expr = M.f_expl_plant;
    end

    sim.solver_options.Tsim            = Tsim;
    sim.solver_options.integrator_type = itype;
    sim.solver_options.num_stages      = nstages;
    sim.solver_options.num_steps       = nsteps;

    % Isolate plant codegen from OCP codegen so that AcadosOcpSolver's
    % post-build cleanup of c_generated_code/ does NOT invalidate the
    % plant's MATLAB class file. Further: each unique plant configuration
    % gets its OWN subdirectory keyed on the resolved settings, so a
    % forced rebuild never has to overwrite a DLL that is still loaded
    % into the current MATLAB session (the Windows linker fails with
    % "Permission denied" on locked DLLs).
    % Keep this path short -- on Windows the combined path
    %   <plant_dir>\CMakeFiles\model_<sim.model.name>.dir\<cmake-hash>\<file>.c.obj.d
    % must stay under MAX_PATH (260 chars). Put the cache outside the repo
    % under cache_root()/plant/<hash> so the full path is short.
    plant_dir = fullfile(cache_root(), 'plant', cfg_hash_short);
    if ~exist(plant_dir, 'dir'); mkdir(plant_dir); end
    sim.code_export_directory = plant_dir;
    addpath(plant_dir);

    plant_solver = AcadosSimSolver(sim);
    cache(key) = plant_solver;
end

% ---------------------------------------------------------------------- %

function [itype, nstages, nsteps] = resolve_plant_integrator(cfg)
% Apply the matched-plant policy with the plant_override escape hatch.
    itype   = cfg.integrator_type;
    nstages = cfg.sim_method_num_stages;
    nsteps  = cfg.sim_method_num_steps;
    if isfield(cfg,'plant_override') && isstruct(cfg.plant_override)
        po = cfg.plant_override;
        if isfield(po,'integrator_type') && ~isempty(po.integrator_type)
            itype = po.integrator_type;
        end
        if isfield(po,'sim_method_num_stages') && ~isempty(po.sim_method_num_stages)
            nstages = po.sim_method_num_stages;
        end
        if isfield(po,'sim_method_num_steps') && ~isempty(po.sim_method_num_steps)
            nsteps = po.sim_method_num_steps;
        end
    end
end
