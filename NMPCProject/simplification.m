% We want to simplify our function
N = 101;
r_range = 0.05;
phi = 0;
rho = linspace(0, r_range, N);
z = linspace(-r_range, r_range, N);

% Avoid singularity at rho=0
rho(1) = rho(2) * 0.1;

I = params.permanent.J / params.physical.mu0 * params.permanent.l(1);
I = -1.2;
r = params.permanent.r(1);
l = params.permanent.l(1);
mu0 = params.physical.mu0;

[RHO, Z] = meshgrid(rho, z);
[bphi, brho, bz] = computeFieldCircularCurrentSheetPolar(phi, RHO, Z, r, l, I, mu0);

% Debug: check for bad values
fprintf('brho: min=%.3e, max=%.3e, NaN=%d, Inf=%d\n', ...
    min(brho(:)), max(brho(:)), sum(isnan(brho(:))), sum(isinf(brho(:))));
fprintf('bz: min=%.3e, max=%.3e, NaN=%d, Inf=%d\n', ...
    min(bz(:)), max(bz(:)), sum(isnan(bz(:))), sum(isinf(bz(:))));

% Replace NaN/Inf with 0 for SVD
brho_clean = brho;
bz_clean = bz;
brho_clean(~isfinite(brho_clean)) = 0;
bz_clean(~isfinite(bz_clean)) = 0;

% Plot field projections
figure;
subplot(2, 2, 1);
plot(rho*1000, brho');
xlabel('\rho [mm]'); ylabel('B_\rho');
title('B_\rho vs \rho (lines = different z)');

subplot(2, 2, 2);
plot(z*1000, brho);
xlabel('z [mm]'); ylabel('B_\rho');
title('B_\rho vs z (lines = different \rho)');

subplot(2, 2, 3);
plot(rho*1000, bz');
xlabel('\rho [mm]'); ylabel('B_z');
title('B_z vs \rho (lines = different z)');

subplot(2, 2, 4);
plot(z*1000, bz);
xlabel('z [mm]'); ylabel('B_z');
title('B_z vs z (lines = different \rho)');

sgtitle('Field projections');

% SVD on cleaned data
[U_rho, S_rho, V_rho] = svd(brho_clean);
[U_z, S_z, V_z] = svd(bz_clean);

s_rho = diag(S_rho);
s_z = diag(S_z);

% Plot SVD spectrum
figure;
subplot(1, 2, 1);
semilogy(s_rho / max(s_rho), 'o-');
xlabel('Index'); ylabel('Normalized magnitude');
title('B_\rho SVD'); grid on;

subplot(1, 2, 2);
semilogy(s_z / max(s_z), 'o-');
xlabel('Index'); ylabel('Normalized magnitude');
title('B_z SVD'); grid on;

% Variance explained
var_rho = sum(s_rho(1:min(3,end)).^2) / sum(s_rho.^2) * 100;
var_z = sum(s_z(1:min(3,end)).^2) / sum(s_z.^2) * 100;
fprintf('B_rho: first 3 singular values explain %.2f%% of variance\n', var_rho);
fprintf('B_z: first 3 singular values explain %.2f%% of variance\n', var_z);

% Determine effective rank
tol = 1e-10;
rank_rho = sum(s_rho / max(s_rho) > tol);
rank_z = sum(s_z / max(s_z) > tol);
fprintf('Effective rank: B_rho=%d, B_z=%d\n', rank_rho, rank_z);

% Test different k values
fprintf('\nApproximation error vs rank:\n');
for k = [3, 5, 10, 15]
    brho_approx = U_rho(:,1:k) * S_rho(1:k,1:k) * V_rho(:,1:k)';
    bz_approx = U_z(:,1:k) * S_z(1:k,1:k) * V_z(:,1:k)';
    err_rho = norm(brho_clean - brho_approx, 'fro') / norm(brho_clean, 'fro');
    err_z = norm(bz_clean - bz_approx, 'fro') / norm(bz_clean, 'fro');
    fprintf('  k=%2d: B_rho=%.2e, B_z=%.2e\n', k, err_rho, err_z);
end

% Final approximation with chosen k
k = 5;
brho_approx = U_rho(:,1:k) * S_rho(1:k,1:k) * V_rho(:,1:k)';
bz_approx = U_z(:,1:k) * S_z(1:k,1:k) * V_z(:,1:k)';

% Plot error vs rank
figure;
max_k = min(50, length(s_rho));
err_vs_k_rho = zeros(max_k, 1);
err_vs_k_z = zeros(max_k, 1);
for kk = 1:max_k
    err_vs_k_rho(kk) = sqrt(sum(s_rho(kk+1:end).^2)) / norm(s_rho);
    err_vs_k_z(kk) = sqrt(sum(s_z(kk+1:end).^2)) / norm(s_z);
end
semilogy(1:max_k, err_vs_k_rho, 'o-', 1:max_k, err_vs_k_z, 's-');
xlabel('Rank k'); ylabel('Relative error');
legend('B_\rho', 'B_z'); grid on;
title('Approximation error vs rank');

% Plot approximation comparison
figure;
subplot(2, 3, 1);
imagesc(rho*1000, z*1000, brho_clean);
colorbar; axis equal tight;
xlabel('\rho [mm]'); ylabel('z [mm]'); title('B_\rho original');

subplot(2, 3, 2);
imagesc(rho*1000, z*1000, brho_approx);
colorbar; axis equal tight;
xlabel('\rho [mm]'); ylabel('z [mm]'); title(sprintf('B_\\rho rank-%d approx', k));

subplot(2, 3, 3);
imagesc(rho*1000, z*1000, brho_clean - brho_approx);
colorbar; axis equal tight;
xlabel('\rho [mm]'); ylabel('z [mm]'); title('B_\rho error');

subplot(2, 3, 4);
imagesc(rho*1000, z*1000, bz_clean);
colorbar; axis equal tight;
xlabel('\rho [mm]'); ylabel('z [mm]'); title('B_z original');

subplot(2, 3, 5);
imagesc(rho*1000, z*1000, bz_approx);
colorbar; axis equal tight;
xlabel('\rho [mm]'); ylabel('z [mm]'); title(sprintf('B_z rank-%d approx', k));

subplot(2, 3, 6);
imagesc(rho*1000, z*1000, bz_clean - bz_approx);
colorbar; axis equal tight;
xlabel('\rho [mm]'); ylabel('z [mm]'); title('B_z error');

sgtitle(sprintf('Rank-%d approximation comparison', k));