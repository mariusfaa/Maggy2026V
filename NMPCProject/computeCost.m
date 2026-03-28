function cost = computeCost(x, u)
% computeCost  Realized stage cost per simulation step.
%
%   cost = computeCost(x, u)
%
%   x    (nx x N)  state error trajectory  (x_sim - xEq)
%   u    (nu x N)  input error trajectory  (u_sim - uEq)
%
%   cost (1 x N)   stage cost at each step:
%                  x_k' * Q * x_k + u_k' * R * u_k
%
%   Q and R are extracted from getCost so cost weights stay in one place.

nx = size(x, 1);
nu = size(u, 1);
cost_obj = getCost(zeros(nx,1), zeros(nu,1));
W  = cost_obj.W;   % blkdiag(Q, R)
Q  = W(1:nx,       1:nx);
R  = W(nx+1:nx+nu, nx+1:nx+nu);

cost = sum((Q * x) .* x, 1) + sum((R * u) .* u, 1);

end
