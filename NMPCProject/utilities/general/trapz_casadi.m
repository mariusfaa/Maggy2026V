function z = trapz_casadi(x, y)
% TRAPZ_CASADI  Trapezoidal integration compatible with CasADi symbolic types.
% Z = TRAPZ_CASADI(X, Y) integrates Y with respect to X.
% X must be a numeric vector of length n.
% Y can be a row vector or matrix of size [m x n].
% Z is a column vector of size [m x 1].
if false
    z = trapz(x,y);
else
    x = x(:);
    n = length(x);
    dx = diff(x);
    
    z = 0;
    for i = 1:n-1
        z = z + dx(i) * (y(:,i) + y(:,i+1)) / 2;
    end
    % z is now [m x 1]
end
end