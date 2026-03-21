%% Generate lookup table for complete elliptic integrals
% Number of samples
N = 10001;   % resolution

% Parameter grid for m = k^2
m_grid = linspace(0,1,N)';  % column vector

% Preallocate arrays
K_vals = zeros(N,1);
E_vals = zeros(N,1);

% Compute elliptic integrals
% ellipke returns K and E for parameter m
[K_vals, E_vals] = ellipke(m_grid);

% Also compute corresponding k = sqrt(m)
k_grid = sqrt(m_grid);

% Save as a MATLAB .mat file
save('ellipticTable.mat','k_grid','m_grid','K_vals','E_vals');

% Save as CSV (columns: k, m, K, E)
T = table(k_grid, m_grid, K_vals, E_vals);
writetable(T,'ellipticTable.csv');

disp('Lookup table saved to ellipticTable.mat and ellipticTable.csv');