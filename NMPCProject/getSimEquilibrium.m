function xEq = getSimEquilibrium(model)
    import casadi.*
    
    z = MX.sym('z', 1, 1);
    
    nx = 10;
    x = MX.zeros(nx, 1);
    x(3) = z;
    u = MX.zeros(4, 1);
    
    dx = substitute(model.f_expl_expr, model.x, x);
    dx = substitute(dx, model.u, u);
    
    cost = dx' * dx;
    
    nlp = struct('x', z, 'f', cost);
    opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
    solver = nlpsol('solver', 'ipopt', nlp, opts);
    
    sol = solver('x0', 0.04);
    
    if full(sol.f) > 1e-10
        error('No equilibrium found.');
    end
    
    xEq = zeros(nx, 1);
    xEq(3) = full(sol.x);
end