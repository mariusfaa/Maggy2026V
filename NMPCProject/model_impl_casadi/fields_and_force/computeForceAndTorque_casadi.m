function [fx,fy,fz,tx,ty,tz] = computeForceAndTorque_casadi(x, u, params, modelId)
% COMPUTEFORCEANDTORQUE_CASADI computes the magnetic force and torque on
% the levitating magnet using CasADi-compatible operations.
%
% Supports two modes:
%   MaglevModel.Accurate (default) - nRadial x nAxial surface discretization
%                                     with double integration
%   MaglevModel.Fast               - nRadial circumferential points only,
%                                     multiply by magnet_l (no axial integration)
%
% Inputs:
%   x       - State vector (12x1, CasADi MX): [pos; euler; vel; omega]
%   u       - Solenoid currents (n_sol x 1, CasADi MX)
%   params  - Parameter struct with params.luts field
%   modelId - MaglevModel.Accurate (default) or MaglevModel.Fast

    import casadi.*

    if nargin < 4, modelId = MaglevModel.Accurate; end

    nRadial  = params.magnet.n;
    magnet_r = params.magnet.r;
    magnet_l = params.magnet.l;
    K = -params.magnet.J / params.physical.mu0;

    %% Rotation matrix from Euler angles
    alpha = x(4);  beta = x(5);  gamma = x(6);

    Rx = [1, 0, 0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha),  cos(alpha)];
    Ry = [cos(beta), 0, sin(beta);
          0, 1, 0;
          -sin(beta), 0, cos(beta)];
    Rz = [cos(gamma), -sin(gamma), 0;
          sin(gamma),  cos(gamma), 0;
          0, 0, 1];
    R = mtimes(mtimes(Rz, Ry), Rx);

    theta = linspace(0, 2*pi - 2*pi/nRadial, nRadial);
    dtheta = 2*pi / nRadial;

    switch modelId
        case MaglevModel.Fast
            %% Fast model: circumference only, multiply by magnet_l
            nEval = nRadial;

            % Surface points at z=0 (midplane)
            px_local = magnet_r * cos(theta);
            py_local = magnet_r * sin(theta);
            pz_local = zeros(1, nEval);

            p_local = [px_local; py_local; pz_local];
            p_global = mtimes(R, p_local);

            px = p_global(1,:) + x(1);
            py = p_global(2,:) + x(2);
            pz = p_global(3,:) + x(3);

            % Compute field
            [bx, by, bz] = computeFieldBase_casadi(px, py, pz, u, params);

            % Force: F = K*l * (tangent x B)
            tangent_local = [cos(theta + pi/2);
                             sin(theta + pi/2);
                             zeros(1, nEval)];
            tangent_global = mtimes(R, tangent_local);
            F = crossProduct3D(K * magnet_l * tangent_global, [bx; by; bz]);

            % Torque: T = K*l * (nvec x B)
            nvec_local = [zeros(2, nEval); ones(1, nEval)];
            nvec_global = mtimes(R, nvec_local);
            T = crossProduct3D(K * magnet_l * nvec_global, [bx; by; bz]);

            % Integrate: periodic rectangle rule (dtheta * sum * r)
            F_sum = magnet_r * dtheta * sum2(F);   % 3x1
            T_sum = magnet_r * dtheta * sum2(T);   % 3x1

            fx = F_sum(1);  fy = F_sum(2);  fz = F_sum(3);
            tx = T_sum(1);  ty = T_sum(2);  tz = T_sum(3);

        otherwise  % MaglevModel.Accurate
            %% Accurate model: nRadial x nAxial surface discretization
            nAxial = params.luts.nAxial;
            nTotal = nRadial * nAxial;

            len = linspace(-magnet_l/2, magnet_l/2, nAxial);

            % Build surface points
            px_local = zeros(1, nTotal);
            py_local = zeros(1, nTotal);
            pz_local = zeros(1, nTotal);
            theta_all = zeros(1, nTotal);

            for j = 1:nAxial
                idx = (j-1)*nRadial + (1:nRadial);
                px_local(idx) = magnet_r * cos(theta);
                py_local(idx) = magnet_r * sin(theta);
                pz_local(idx) = len(j);
                theta_all(idx) = theta;
            end

            p_local = [px_local; py_local; pz_local];
            p_global = mtimes(R, p_local);

            px = p_global(1,:) + x(1);
            py = p_global(2,:) + x(2);
            pz = p_global(3,:) + x(3);

            % Compute field
            [bx, by, bz] = computeFieldBase_casadi(px, py, pz, u, params);

            % Force: F = K * (tangent x B)
            tangent_local = [cos(theta_all + pi/2);
                             sin(theta_all + pi/2);
                             zeros(1, nTotal)];
            tangent_global = mtimes(R, tangent_local);
            F = crossProduct3D(K * tangent_global, [bx; by; bz]);

            % Torque: T = K * (nvec x B)
            nvec_local = [zeros(2, nTotal); ones(1, nTotal)];
            nvec_global = mtimes(R, nvec_local);
            T = crossProduct3D(K * nvec_global, [bx; by; bz]);

            % Double integration
            F_cols = cell(1, nAxial);
            T_cols = cell(1, nAxial);
            for j = 1:nAxial
                idx = (j-1)*nRadial + (1:nRadial);
                F_cols{j} = magnet_r * dtheta * sum2(F(:, idx));
                T_cols{j} = magnet_r * dtheta * sum2(T(:, idx));
            end

            inner_F = horzcat(F_cols{:});
            inner_T = horzcat(T_cols{:});

            fx = trapzVec(len, inner_F(1,:));
            fy = trapzVec(len, inner_F(2,:));
            fz = trapzVec(len, inner_F(3,:));
            tx = trapzVec(len, inner_T(1,:));
            ty = trapzVec(len, inner_T(2,:));
            tz = trapzVec(len, inner_T(3,:));
    end
end

%% Cross product for 3xN matrices
function c = crossProduct3D(a, b)
    c = [a(2,:) .* b(3,:) - a(3,:) .* b(2,:);
         a(3,:) .* b(1,:) - a(1,:) .* b(3,:);
         a(1,:) .* b(2,:) - a(2,:) .* b(1,:)];
end

%% Trapezoidal integration (non-periodic, for axial direction)
function z = trapzVec(x_vec, y)
    N = length(x_vec);
    dx = x_vec(2) - x_vec(1);
    z = dx * (y(1) + y(2)) / 2;
    for i = 2:N-1
        dx = x_vec(i+1) - x_vec(i);
        z = z + dx * (y(i) + y(i+1)) / 2;
    end
end
