function [Ad, Bd] = getLinsys(x, u, dt_mpc)
    import casadi.*
    nx = 10;
    nu = 4;

    model = getSimModel(32);
    x_cas = model.x;      % use the model's own symbolic variables
    u_cas = model.u;
    f_expl = model.f_expl_expr;

    Ac_sym = jacobian(f_expl, x_cas);
    Bc_sym = jacobian(f_expl, u_cas);

    Ac_fn = Function('Ac', {x_cas, u_cas}, {Ac_sym});
    Bc_fn = Function('Bc', {x_cas, u_cas}, {Bc_sym});

    % evaluate numerically at the operating point
    Ac = full(Ac_fn(x, u));
    Bc = full(Bc_fn(x, u));

    % exact discretization via matrix exponential
    M  = expm([Ac, Bc; zeros(nu, nx+nu)] * dt_mpc);
    Ad = M(1:nx, 1:nx);
    Bd = M(1:nx, nx+1:nx+nu);
end