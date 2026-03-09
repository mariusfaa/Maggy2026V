function Pi = ellipP_casadi(h2, k2)
% ELLIPP_CASADI  Complete elliptic integral of the third kind (CasADi-compatible).
%   Uses 10-point Gauss-Legendre quadrature, matching ellipP.m but with
%   CasADi-compatible operations (no indexing, no conditionals).
%
%   Pi(h2, k2) = int(1/((1 - h2*sin(t)^2)*sqrt(1 - k2*sin(t)^2)), t=0..pi/2)
%
%   Inputs:
%     h2 - characteristic parameter (1 x N, CasADi MX or double)
%     k2 - modulus squared (1 x N, CasADi MX or double)
%
%   Output:
%     Pi - elliptic integral of third kind (1 x N)

    import casadi.*

    % Gauss-Legendre 10-point quadrature nodes and weights
    t = [0.9931285991850949,  0.9639719272779138, ...
         0.9122344282513259,  0.8391169718222188, ...
         0.7463319064601508,  0.6360536807265150, ...
         0.5108670019508271,  0.3737060887154195, ...
         0.2277858511416451,  0.07652652113349734];
    w = [0.01761400713915212, 0.04060142980038694, ...
         0.06267204833410907, 0.08327674157670475, ...
         0.1019301198172404,  0.1181945319615184, ...
         0.1316886384491766,  0.1420961093183820, ...
         0.1491729864726037,  0.1527533871307258];

    P = 0;
    for i = 1:10
        c0 = pi * t(i) / 4;
        u_plus  = pi/4 + c0;
        u_minus = pi/4 - c0;

        sn2_plus  = sin(u_plus).^2;
        sn2_minus = sin(u_minus).^2;

        g_plus  = 1 ./ ((1 - h2 .* sn2_plus)  .* sqrt(1 - k2 .* sn2_plus));
        g_minus = 1 ./ ((1 - h2 .* sn2_minus) .* sqrt(1 - k2 .* sn2_minus));

        P = P + w(i) .* (g_plus + g_minus);
    end

    Pi = (pi/4) * P;
end
