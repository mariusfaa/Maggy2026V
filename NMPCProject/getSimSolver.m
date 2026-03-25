function sim_solver = getSimSolver(model, dt)
% Return an acados sim solver object that uses the most accurate maglev
% model.

% Setup sim solver object
fprintf('--- Building sim solver ---');

sim = AcadosSim();
sim.model = model;
sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;
sim_solver = AcadosSimSolver(sim);

end