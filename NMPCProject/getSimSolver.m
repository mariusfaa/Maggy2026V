function sim_solver = getSimSolver(model, dt)
% Return an acados sim solver object that uses the most accurate maglev
% model.

% Setup sim solver object
fprintf('--- Building sim solver ---\n');

sim = AcadosSim();
sim.model = model;
sim.solver_options.Tsim            = dt;
sim.solver_options.integrator_type = 'ERK';
sim.solver_options.num_stages      = 4;
sim.solver_options.num_steps       = 1;
solver_dir = fullfile('build', 'sim');
sim.code_gen_opts.code_export_directory = fullfile(solver_dir, 'c_generated_code');
sim.code_gen_opts.json_file = fullfile(solver_dir, [model.name '_sim.json']);
sim_solver = AcadosSimSolver(sim, struct('output_dir', solver_dir));

end