%% validate_model.m  —  Track CasADi expression graph complexity
%
% Builds the maglev model, computes the Jacobians that acados needs for
% MPC with ERK integrator (Gauss-Newton Hessian), and reports
% n_instructions for each.
%
% What acados actually evaluates per SQP iteration (ERK + Gauss-Newton):
%   f_expl  — ODE right-hand side
%   df/dx   — state Jacobian (forward sensitivity, gives A matrix)
%   df/du   — input Jacobian (forward sensitivity, gives B matrix)
%
% Adjoint/Hessian are NOT used with Gauss-Newton approximation.
%
% Results are appended to model_complexity.csv with a user comment.
%
% Usage:
%   validate_model("baseline before any optimisation")
%   validate_model("baked perm constants in computeFieldBase")

function validate_model(comment, modelId)
    if nargin < 1
        comment = input('Comment for this run: ', 's');
    end
    if nargin < 2
        modelId = MaglevModel.Fast;
    end

    import casadi.*

    %% Build model
    fprintf('=== Building model (%s) ===\n', string(modelId));
    model = get_maggy_model(modelId, use_luts=false);

    x      = model.x;
    u      = model.u;
    f_expl = model.f_expl_expr;

    nx = size(x, 1);
    nu = size(u, 1);

    %% 1. f_expl — ODE evaluation
    f_fun = Function('f_expl', {x, u}, {f_expl});
    n_f = f_fun.n_instructions();

    fprintf('\n=== Expression graph complexity (MPC-relevant) ===\n');
    fprintf('  f_expl            : %6d instructions\n', n_f);

    %% 2. df/dx — state Jacobian (linearisation A matrix)
    jac_x = jacobian(f_expl, x);
    f_jac_x = Function('df_dx', {x, u}, {jac_x});
    n_jac_x = f_jac_x.n_instructions();
    fprintf('  df/dx  (%2dx%2d)    : %6d instructions\n', nx, nx, n_jac_x);

    %% 3. df/du — input Jacobian (linearisation B matrix)
    jac_u = jacobian(f_expl, u);
    f_jac_u = Function('df_du', {x, u}, {jac_u});
    n_jac_u = f_jac_u.n_instructions();
    fprintf('  df/du  (%2dx%2d)     : %6d instructions\n', nx, nu, n_jac_u);

    %% Summary
    n_total = n_f + n_jac_x + n_jac_u;
    fprintf('  ─────────────────────────────────\n');
    fprintf('  TOTAL             : %6d instructions\n', n_total);

    %% Append to CSV
    csv_file = fullfile(fileparts(mfilename('fullpath')), 'model_complexity.csv');
    write_header = ~isfile(csv_file);

    fid = fopen(csv_file, 'a');
    if write_header
        fprintf(fid, 'timestamp,comment,f_expl,df_dx,df_du,total\n');
    end
    ts = string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss'));
    comment_safe = strrep(comment, '"', '""');
    fprintf(fid, '%s,"%s",%d,%d,%d,%d\n', ...
        ts, comment_safe, n_f, n_jac_x, n_jac_u, n_total);
    fclose(fid);

    fprintf('\nResults appended to %s\n', csv_file);
end
