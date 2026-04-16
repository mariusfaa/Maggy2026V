function [K, E] = ellipke_casadi(m)
% ELLIPKE_CASADI  Complete elliptic integrals K(m) and E(m) for CasADi.
%
%   [K, E] = ellipke_casadi(m)
%
%   Uses the Arithmetic-Geometric Mean (AGM) iteration, which converges
%   quadratically and achieves near machine precision (~1e-15) in 6
%   iterations.  All operations (+, -, *, /, sqrt) are natively supported
%   and automatically differentiable in CasADi.
%
%   Reference: Abramowitz & Stegun, Section 17.6
%              Borwein & Borwein, "Pi and the AGM" (1987)
%
% Input:
%   m  - Elliptic integral parameter, 0 <= m < 1 (CasADi SX/MX)
%
% Outputs:
%   K  - Complete elliptic integral of the first kind
%   E  - Complete elliptic integral of the second kind

    import casadi.*

    % Clamp m to [0, 1-eps] for numerical stability
    m = fmin(fmax(m, 0), 1 - 1e-9);

    % AGM initial values
    a = 1 + 0*m;       % Promote to SX if m is symbolic
    b = sqrt(1 - m);

    % Running sum for E(m) via Gauss formula (A&S 17.6):
    %   E = K * (1 - sum),  sum = sum_{i=0}^{N} 2^{i-1} * c_i^2
    sum_c2 = m / 2;    % i=0 term: 2^{-1} * c_0^2 = m/2
    pow2   = 1;         % 2^{i-1} for i=1

    % 6 AGM iterations (quadratic convergence => ~10^19 digits)
    for i = 1:6
        c      = (a - b) / 2;
        sum_c2 = sum_c2 + pow2 * c.^2;
        pow2   = pow2 * 2;
        a_new  = (a + b) / 2;
        b      = sqrt(a .* b);
        a      = a_new;
    end

    K = pi ./ (2 * a);
    E = K .* (1 - sum_c2);
end
