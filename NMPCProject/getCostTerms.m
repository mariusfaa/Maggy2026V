function cost = getCostTerms(xEq,uEq,dt_mpc,use_dare)
    arguments
        xEq (:,1) double % Required argument
        uEq (:,1) double = zeros(4,1) % Optional with default
        dt_mpc double = 0.001
        use_dare logical = false
    end

cost = struct();


nx = 10;
nu = 4;

cost.Q = diag([...
    1e6, 1e6, ...       % x, y (lateral — less critical)
    1e7, ...             % z (unstable direction — highest priority)
    1e2, 1e2, ...       % roll, pitch
    1e0, 1e0, 1e0, ...  % vx, vy, vz (vz high for damping)
    1e1, 1e1 ...        % wx, wy
]);
cost.R = eye(nu) * 1e0;

if use_dare
    % terminal cost from DARE, a solution P to
    % A'(P-PB((R+B'PB)exp-1*B'P)A+Q-P=0
    [Ad, Bd, ~] = lsys(xEq, uEq);
    Ad = full(Ad); Bd = full(Bd);
    [~, P, ~] = dlqr(Ad, Bd, cost.Q, cost.R);
    cost.Qt = P;
end

end