%% Build grid (half-plane, z>=0)
rho_max = 0.1;% max length in xy plane
z_max = 0.06; % +-6cm

N_rho = 50;
N_z   = 50;
rho_vec = linspace(0, rho_max, N_rho);
z_vec   = linspace(0, z_max,  N_z);

[RHO, ZZ] = meshgrid(rho_vec, z_vec);

BRHO = zeros(size(RHO));
BZ   = zeros(size(RHO));

for k = 1:numel(RHO)
    rho_k = RHO(k);
    z_k   = ZZ(k);
    if z_k == 0 && rho_k < r  % on-axis edge case if needed
        continue
    end
    [brho_k, bz_k] = computeFieldCircularCurrentSheetPolarSmart(rho_k, z_k, r, l, mu0);
    BRHO(k) = brho_k * I;
    BZ(k)   = bz_k   * I;
end

%% CasADi interpolants
% CasADi expects data in column-major order matching meshgrid layout
import casadi.*

lut_brho = interpolant('lut_brho', 'bspline', {z_vec, rho_vec}, BRHO(:)');
lut_bz   = interpolant('lut_bz',   'bspline', {z_vec, rho_vec}, BZ(:)');

%% Wrap with symmetry in a CasADi Function
rho_sym = MX.sym('rho');
z_sym   = MX.sym('z');

z_abs  = abs(z_sym);
brho_val = lut_brho([z_abs; rho_sym]);
bz_val   = lut_bz(  [z_abs; rho_sym]);

% Apply z-symmetry
brho_out = if_else(z_sym < 0, -brho_val, brho_val);
bz_out   =  bz_val;  % even in z, no sign flip needed

F_field = Function('F_field', {rho_sym, z_sym}, {brho_out, bz_out}, ...
                   {'rho','z'}, {'brho','bz'});