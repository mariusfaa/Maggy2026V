function [K, E] = ellipke_casadi2(m)
    % ELLIPKE_CASADI Compute complete elliptic integrals of first and second kind
    % using CasADi-compatible operations (Hastings approximation).
    %
    % Inputs:
    %   m - Parameter (k^2), where 0 <= m < 1.
    %
    % Outputs:
    %   K - Complete elliptic integral of the first kind.
    %   E - Complete elliptic integral of the second kind.
    
    import casadi.*
    
    % Ensure m is within [0, 1) to avoid singularities and complex numbers
    % In CasADi, use fmin/fmax for smooth min/max operations if needed, 
    % generally simple constraints on the OCP states prevent m > 1.
    % For safety inside the function:
    % m = fmin(fmax(m, 0), 1 - 1e-9);

    % Complementary parameter
    m1 = 1 - m;
    
    % Log term
    a0 = 1.38629436112;
    a1 = 0.09666344259;
    a2 = 0.03590092383;
    a3 = 0.03742563713;
    a4 = 0.01451196212;

    b0 = 0.5;
    b1 = 0.12498593597;
    b2 = 0.06880248576;
    b3 = 0.03328355346;
    b4 = 0.00441787012;
    
    % Approximation for K
    K_poly1 = (((a4*m1 + a3).*m1 + a2).*m1 + a1).*m1 + a0;
    K_poly2 = (((b4*m1 + b3).*m1 + b2).*m1 + b1).*m1 + b0;
    K = K_poly1 - K_poly2 .* log(m1);

    % Coefficients for E
    c0 = 1;
    c1 = 0.44325141463;
    c2 = 0.06260601220;
    c3 = 0.04757383546;
    c4 = 0.01736506451;

    d0 = 0; % Note: d0 is 0 in the standard approximation
    d1 = 0.24998368310;
    d2 = 0.09200180037;
    d3 = 0.04069697526;
    d4 = 0.00526449639;

    % Approximation for E
    E_poly1 = (((c4*m1 + c3).*m1 + c2).*m1 + c1).*m1 + c0;
    E_poly2 = (((d4*m1 + d3).*m1 + d2).*m1 + d1).*m1 + d0;
    E = E_poly1 - E_poly2 .* log(m1);
end