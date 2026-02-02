function [bx, by, bz] = casadiComputeFieldBase(x, y, z, u, params)
    % A simplified, CasADi-compatible version of computeFieldBase
    % Assumes 'fast' model (wire loops) for efficiency.
    
    import casadi.*

    % Initialize accumulator
    bx = 0; by = 0; bz = 0;

    % 1. Field from Permanent Magnets (Base)
    for i = 1:length(params.permanent.r)
        % Current equivalent for PM
        I_pm = params.permanent.J / params.physical.mu0 * params.permanent.l(i);
        
        % Relative distances
        dx = x - params.permanent.x(i);
        dy = y - params.permanent.y(i);
        dz = z - params.permanent.z(i);
        
        [bx_temp, by_temp, bz_temp] = casadiWireField(dx, dy, dz, params.permanent.r(i), I_pm, params.physical.mu0);
        
        bx = bx + bx_temp;
        by = by + by_temp;
        bz = bz + bz_temp;
    end

    % 2. Field from Solenoids
    for i = 1:length(params.solenoids.r)
        % Solenoid Current (Control Input u)
        I_sol = u(i); 
        
        dx = x - params.solenoids.x(i);
        dy = y - params.solenoids.y(i);
        dz = z - params.solenoids.z(i);

        [bx_temp, by_temp, bz_temp] = casadiWireField(dx, dy, dz, params.solenoids.r(i), I_sol, params.physical.mu0);
        
        % Multiply by number of windings for the 'fast' approximation
        bx = bx + bx_temp * params.solenoids.nw;
        by = by + by_temp * params.solenoids.nw;
        bz = bz + bz_temp * params.solenoids.nw;
    end
end