function [bx, by, bz] = casadiWireField(x, y, z, r, I, mu0)
    % CasADi implementation of computeFieldCircularWirePolar
    % combined with cart2pol and pol2cart logic.

    import casadi.*
    
    % Cartesian to Polar
    rho = sqrt(x.^2 + y.^2);
    % Avoid division by zero in phi calculation (though phi cancels out in bz, it matters for bx/by)
    % We use cos(phi) = x/rho, sin(phi) = y/rho directly later to avoid atan2 singularities
    
    % Field Calculation Logic from computeFieldCircularWirePolar.m
    c = mu0 * I ./ (4 * pi * sqrt(r * rho));
    
    % The parameter m for elliptic integrals (k^2 in the original code)
    m = (4 * r * rho) ./ ((r + rho).^2 + z.^2);
    
    % Use our custom approximation
    [K, E] = ellipke_casadi(m);
    
    % Radial and Axial components
    % Note: Added small epsilon to denominator to prevent division by zero when rho=r and z=0
    eps = 1e-9; 
    denom = (rho - r).^2 + z.^2 + eps;
    
    brho = -(z ./ (rho + eps)) .* c .* sqrt(m) .* (K - (rho.^2 + r^2 + z.^2) ./ denom .* E);
    bz_pol = c .* sqrt(m) .* (K - (rho.^2 - r^2 + z.^2) ./ denom .* E);
    
    % Polar to Cartesian projection
    % bx = brho * cos(phi) => brho * (x / rho)
    % by = brho * sin(phi) => brho * (y / rho)
    
    bx = brho .* (x ./ (rho + eps));
    by = brho .* (y ./ (rho + eps));
    bz = bz_pol;
end