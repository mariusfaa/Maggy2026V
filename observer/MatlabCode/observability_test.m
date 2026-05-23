
Ts = 0.002; % 500 Hz
T = 12*Ts; % all states times discretization time
dt_int = 1e-6; % integration step for continuous approx

systems = {
    struct('A', A,     'C', C,     'name', '12-state'),
    struct('A', Ared,  'C', Cred,  'name', '10-state'),
    struct('A', Axred, 'C', Cxred, 'name', '6-state')
};

results = struct();

%% ===== CONTINUOUS-TIME ANALYSIS =====
for i = 1:length(systems)
    sys = systems{i};
    
    Wo = finite_horizon_gramian_ct(sys.A, sys.C, T, dt_int);
    
    eigvals = eig((Wo + Wo')/2); % enforce symmetry numerically
    eigvals = real(eigvals);
    
    results(i).name = sys.name;
    results(i).Wo_ct = Wo;
    results(i).eig_ct = sort(eigvals, 'descend');
    results(i).cond_ct = max(eigvals) / max(min(eigvals), 1e-16);
    results(i).rank_ct = rank(Wo);
end

%% ===== DISCRETE-TIME ANALYSIS =====
fs = 1/Ts;  % sampling frequency (Hz)
N = round(T / Ts);

for i = 1:length(systems)
    sys = systems{i};

    Ad = expm(sys.A * Ts);  % ZOH discretization
    Cd = sys.C;

    Wo = finite_horizon_gramian_dt(Ad, Cd, N);

    eigvals = eig((Wo + Wo')/2);
    eigvals = real(eigvals);

    results(i).eig_dt = sort(eigvals, 'descend');

    results(i).cond_dt = max(eigvals) / max(min(eigvals), 1e-16);

    results(i).rank_dt = rank(Wo);
end


%% plotting
figure; tiledlayout(1,3);

for i = 1:length(results)
    nexttile; hold on;
    
    eigvals = results(i).eig_ct;
    
    eigvals = real(eigvals);
    eigvals = eigvals(isfinite(eigvals));
    eigvals = sort(eigvals, 'descend');
    
    %eigvals = eigvals / max(eigvals);
    %eigvals(eigvals < 1e-16) = 1e-16;
    
    stem(eigvals, 'filled', 'LineWidth', 1.2);
    set(gca, 'YScale', 'log');
    
    xlabel('Eigenvalue index');
    ylabel('Eigenvalue');

    title(results(i).name);
    
    ylim([1e-20 1]);
end

sgtitle('Continuous-Time Observability Spectrum (Stem Plot)');
%saveas(gcf, 'obsv_spectra_cont.png');

%%
figure; tiledlayout(1, length(results));

for i = 1:length(results)
    nexttile; hold on;
    
    eigvals = results(i).eig_dt;
    
    eigvals = real(eigvals);
    eigvals = eigvals(isfinite(eigvals));
    eigvals = sort(eigvals, 'descend');
    
    %eigvals = eigvals / max(eigvals);
    %eigvals(eigvals < 1e-16) = 1e-16;
    
    stem(eigvals, 'filled');
    set(gca, 'YScale', 'log');
    
    title(results(i).name);
    xlabel('Eigenvalue index');
    ylabel('Eigenvalue');

    ylim([1e-18 1e+2]);
end

sgtitle(['Discrete Observability Spectrum (' num2str(fs) ' Hz)']);
saveas(gcf, 'obsv_spectra_disc.png');


%% functions
function Wo = finite_horizon_gramian_ct(A, C, T, dt)
    n = size(A,1);
    Wo = zeros(n);
    
    t_vals = 0:dt:T;
    
    for k = 1:length(t_vals)
        t = t_vals(k);
        E = expm(A * t);
        Wo = Wo + (E' * (C' * C) * E) * dt;
    end
end

function Wo = finite_horizon_gramian_dt(Ad, C, N)
    n = size(Ad,1);
    Wo = zeros(n);
    
    Ak = eye(n);
    
    for k = 0:N-1
        Wo = Wo + (Ak' * (C' * C) * Ak);
        Ak = Ad * Ak;
    end
end