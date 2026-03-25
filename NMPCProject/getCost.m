function cost = getCost(xEq,uEq)
    arguments
        xEq (:,1) double % Required argument
        uEq (:,1) double = zeros(4,1) % Optional with default
    end

% Return ocp cost object that is persistant for all models

import casadi.*

nx = 10;
nu = 4;

Q = diag([...
    1e4, 1e4, ...       % x, y (lateral — less critical)
    1e6, ...             % z (unstable direction — highest priority)
    1e3, 1e3, ...       % roll, pitch
    1e2, 1e2, 1e3, ...  % vx, vy, vz (vz high for damping)
    1e2, 1e2 ...        % wx, wy
]);
R = eye(nu) * 1e0;

cost = AcadosOcpCost();

cost.cost_type   = 'LINEAR_LS';
cost.cost_type_0 = 'LINEAR_LS';
cost.cost_type_e = 'LINEAR_LS';

cost.Vx   = [eye(nx); zeros(nu, nx)];
cost.Vu   = [zeros(nx, nu); eye(nu)];
cost.Vx_0 = cost.Vx;
cost.Vu_0 = cost.Vu;
cost.Vx_e = eye(nx);

cost.W   = blkdiag(Q, R);
cost.W_0 = blkdiag(Q, R);
cost.W_e = 10 * Q;  % heavier terminal cost for unstable system

cost.yref   = [xEq; uEq];
cost.yref_0 = [xEq; uEq];
cost.yref_e = xEq;

end