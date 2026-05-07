function plant_solver = build_plant_v2(plant_n, dt, M)
% BUILD_PLANT_V2  Build (or fetch from cache) the 12-state plant simulator.
%
%   plant_solver = build_plant_v2(plant_n, dt, M)
%
% Cached on (plant_n, round(dt*1e6)) inside this MATLAB session via a
% persistent map. The plant is *always* 12-state and uses M.f_expl_plant
% which was built with the appropriate magnet.n.
%
% Inputs
%   plant_n   discretization (typically 30)
%   dt        Tsim for one call to the plant solver, equals controller's Tf/N
%   M        struct from build_model_v2 (uses M.x_plant_sym, M.u_sym,
%             M.xdot_plant_sym, M.f_expl_plant)
%
% Output
%   plant_solver  AcadosSimSolver (12-state, IRK 4 stages / 10 steps)

    persistent cache
    if isempty(cache)
        cache = containers.Map('KeyType','char','ValueType','any');
    end

    key = sprintf('n%d_dt%d', plant_n, round(dt*1e6));
    if isKey(cache, key)
        plant_solver = cache(key);
        return;
    end

    sim = AcadosSim();
    sim.model.name        = sprintf('plant_n%d_dt%d', plant_n, round(dt*1e6));
    sim.model.x           = M.x_plant_sym;
    sim.model.u           = M.u_sym;
    sim.model.xdot        = M.xdot_plant_sym;
    sim.model.f_impl_expr = M.xdot_plant_sym - M.f_expl_plant;
    sim.solver_options.Tsim            = dt;
    sim.solver_options.integrator_type = 'IRK';
    sim.solver_options.num_stages      = 4;
    sim.solver_options.num_steps       = 10;

    plant_solver = AcadosSimSolver(sim);
    cache(key) = plant_solver;
end
