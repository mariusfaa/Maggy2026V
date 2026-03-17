function [fx,fy,fz,tx,ty,tz] = computeForceAndTorque_casadi(x, u, params, modelId)
% COMPUTEFORCEANDTORQUE_CASADI  Reduced 10-state force/torque (Fast model only).
%
% R = Ry(pitch) * Rx(roll), no Rz (yaw removed).
% Wire-loop field model: circumferential points only, multiply by magnet_l.
%
% Inputs:
%   x       - State vector (10x1): [x;y;z; roll;pitch; vx;vy;vz; wx;wy]
%   u       - Solenoid currents (n_sol x 1)
%   params  - Parameter struct
%   modelId - MaglevModel.Fast

    import casadi.*

    assert(numel(x) == 10);

    nRadial  = params.magnet.n;
    magnet_r = params.magnet.r;
    magnet_l = params.magnet.l;
    K = -params.magnet.J / params.physical.mu0;

    %% Rotation matrix from Euler angles
    alpha = x(4);  beta = x(5);

    Rx = [1, 0, 0;
          0, cos(alpha), -sin(alpha);
          0, sin(alpha),  cos(alpha)];
    Ry = [cos(beta), 0, sin(beta);
          0, 1, 0;
          -sin(beta), 0, cos(beta)];
    % Rz is ignored since yaw is removed from this model
    R = mtimes(Ry, Rx);

    theta = linspace(0, 2*pi - 2*pi/nRadial, nRadial);
    dtheta = 2*pi / nRadial;

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
    if params.lut_opts.enabled
        [bx, by, bz] = computeFieldBase_lut(px, py, pz, u, params);
    else
        [bx, by, bz] = computeFieldBase_analytical(px, py, pz, u, params, modelId);
    end

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
end

%% Cross product for 3xN matrices
function c = crossProduct3D(a, b)
    c = [a(2,:) .* b(3,:) - a(3,:) .* b(2,:);
         a(3,:) .* b(1,:) - a(1,:) .* b(3,:);
         a(1,:) .* b(2,:) - a(2,:) .* b(1,:)];
end