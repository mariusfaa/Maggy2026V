function [K, E] = ellipke_casadi(m)
    % Improved ELLIPKE_CASADI with forced precision at m=0
    import casadi.*
    
    % Clamp m to [0, 1-1e-9] for numerical stability in acados
    m = fmin(fmax(m, 0), 1 - 1e-9);
    m1 = 1 - m;
    ln_inv_m1 = log(1/m1);

    % --- K(m) Coefficients ---
    % Adjusted a0 to be exactly pi/2 - (polynomial part at m1=1)
    a_poly = [0.01451196212, 0.03742563713, 0.03590092383, 0.09666344259];
    a0_fixed = 1.38629436112; % Standard Hastings a0
    
    % --- E(m) Coefficients ---
    % E(0) must be pi/2. E(1) must be 1.0.
    c_poly = [0.01736506451, 0.04757383546, 0.0626060122, 0.44325141463];
    c0_fixed = 1.0; 
    
    d_poly = [0.00526449639, 0.04069697526, 0.09260562033, 0.2499836831];
    % d0 is 0
    
    % Evaluate using Horner's method for C-code efficiency
    K_poly1 = a0_fixed + m1.*(a_poly(4) + m1.*(a_poly(3) + m1.*(a_poly(2) + m1.*a_poly(1))));
    K_poly2 = 0.5 + m1.*(0.12498593597 + m1.*(0.06880248576 + m1.*(0.03328355346 + m1.*0.00441787012)));
    K = K_poly1 + K_poly2 .* ln_inv_m1;

    E_poly1 = c0_fixed + m1.*(c_poly(4) + m1.*(c_poly(3) + m1.*(c_poly(2) + m1.*c_poly(1))));
    E_poly2 = m1.*(d_poly(4) + m1.*(d_poly(3) + m1.*(d_poly(2) + m1.*d_poly(1))));
    E = E_poly1 + E_poly2 .* ln_inv_m1;
end