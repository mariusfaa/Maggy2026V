function [bx, by, bz] = computeFieldBase_casadi(px, py, pz, u, params)
% COMPUTEFIELDBASE_CASADI computes the magnetic field from the base
% (permanent magnets + solenoids) at evaluation points using CasADi LUTs.
%
% Inputs:
%   px, py, pz - Evaluation point coordinates (1 x nEval, CasADi MX)
%   u          - Solenoid currents (n_sol x 1, CasADi MX)
%   params     - Parameter struct with params.luts field
%
% Outputs:
%   bx, by, bz - Magnetic field components (1 x nEval, CasADi MX)

    import casadi.*

    nEval = params.luts.nEval;
    n_sol = length(params.solenoids.r);
    eps_val = 1e-9;

    %% Permanent magnets - 3D Cartesian LUT (one point at a time)
    bx_parts = cell(1, nEval);
    by_parts = cell(1, nEval);
    bz_parts = cell(1, nEval);
    for k = 1:nEval
        res = params.luts.perm_3d([px(k); py(k); pz(k)]);
        bx_parts{k} = res(1);
        by_parts{k} = res(2);
        bz_parts{k} = res(3);
    end
    bx = horzcat(bx_parts{:});
    by = horzcat(by_parts{:});
    bz = horzcat(bz_parts{:});

    %% Solenoids - 2D polar LUT (per-source, one point at a time)
    for i = 1:n_sol
        dx = px - params.solenoids.x(i);
        dy = py - params.solenoids.y(i);
        dz = pz - params.solenoids.z(i);
        rho = sqrt(dx.^2 + dy.^2);

        brho_parts = cell(1, nEval);
        bzf_parts  = cell(1, nEval);
        for k = 1:nEval
            res = params.luts.sol_field([dz(k); rho(k)]);
            brho_parts{k} = res(1);
            bzf_parts{k}  = res(2);
        end
        brho = horzcat(brho_parts{:});
        bz_f = horzcat(bzf_parts{:});

        % Polar -> Cartesian, scale by current
        bx = bx + brho .* (dx ./ (rho + eps_val)) * u(i);
        by = by + brho .* (dy ./ (rho + eps_val)) * u(i);
        bz = bz + bz_f * u(i);
    end
end
