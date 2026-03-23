function [bx_s, by_s, bz_s] = computeFieldDipole_casadi(px_s, py_s, pz_s, ...
                                                         src_x, src_y, src_z, ...
                                                         mx, my, mz, mu0)
% COMPUTEFIELDDIPOLE_CASADI  Magnetic field of a dipole source (SX).
%
% Computes the field of a magnetic dipole with moment [mx,my,mz] located
% at [src_x, src_y, src_z] evaluated at field point [px_s, py_s, pz_s].
%
% B(r) = (mu0 / 4*pi) * [ 3*(m.rhat)*rhat - m ] / |r|^3
%
% All inputs can be CasADi SX or numeric. Source position and moment are
% expected to be numeric constants (baked in at model creation).

    import casadi.*

    eps_val = 1e-12;

    % Displacement from source to field point
    dx = px_s - src_x;
    dy = py_s - src_y;
    dz = pz_s - src_z;

    r2 = dx^2 + dy^2 + dz^2 + eps_val;
    r  = sqrt(r2);
    r5 = r2^2 * r;   % r^5

    % m dot r
    m_dot_r = mx*dx + my*dy + mz*dz;

    % Dipole field: (mu0/4pi) * (3*(m.r)*r/r^5 - m/r^3)
    coeff = mu0 / (4 * pi);

    bx_s = coeff * (3 * m_dot_r * dx / r5 - mx / (r2*r));
    by_s = coeff * (3 * m_dot_r * dy / r5 - my / (r2*r));
    bz_s = coeff * (3 * m_dot_r * dz / r5 - mz / (r2*r));
end
