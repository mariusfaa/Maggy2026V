%% =========================================================================
%  Observability Gramian Analysis
%  Requires: A, C, Ared, Cred, Axred, Cxred already in workspace
%% =========================================================================

%% ===== PARAMETERS (all tuneable here) ====================================
Ts        = 0.005;    % 200 Hz sampling period
T         = 4*Ts;     % analysis horizon for main gramians
dt_int    = 1e-6;     % integration step for CT gramian  (coarsen if slow)
T_stop    = 0.1;      % condition-number sweep end time [s]
dt_sweep  = 1e-6;     % integration step for condition-number sweep

systems = {
    struct('A', A,     'C', C,     'name', '12-state'), ...
    struct('A', Ared,  'C', Cred,  'name', '10-state'), ...
    struct('A', Axred, 'C', Cxred, 'name', '6-state')
};
results = struct();

%% ===== CONTINUOUS-TIME OBSERVABILITY GRAMIANS ============================
for i = 1:length(systems)
    sys  = systems{i};
    Wo   = finite_horizon_gramian_ct(sys.A, sys.C, T, dt_int);
    eigs = real(eig((Wo + Wo') / 2));
    results(i).name    = sys.name;
    results(i).Wo_ct   = Wo;
    results(i).eig_ct  = sort(eigs, 'descend');
    results(i).cond_ct = max(eigs) / min(eigs);
    results(i).rank_ct = rank(Wo);
end

%% ===== DISCRETE-TIME OBSERVABILITY GRAMIANS ==============================
fs = 1/Ts;
N  = round(T / Ts);
for i = 1:length(systems)
    sys  = systems{i};
    Ad   = expm(sys.A * Ts);
    Wo   = finite_horizon_gramian_dt(Ad, sys.C, N);
    eigs = real(eig((Wo + Wo') / 2));
    results(i).eig_dt  = sort(eigs, 'descend');
    results(i).cond_dt = max(eigs) / min(eigs);
    results(i).rank_dt = rank(Wo);
end

%% ===== CONDITION NUMBER OVER TIME ========================================
for i = 1:length(systems)
    sys = systems{i};
    Ad  = expm(sys.A * Ts);
    [t_ct, kct, t_dt, kdt] = cond_vs_time( ...
        sys.A, sys.C, Ad, Ts, T_stop, dt_sweep);
    results(i).t_ct     = t_ct;
    results(i).kappa_ct = kct;
    results(i).t_dt     = t_dt;
    results(i).kappa_dt = kdt;
end

%% ===== CSV EXPORT ========================================================
% --- Eigenvalues (long format) ---
fid = fopen('obsv_eigenvalues.csv', 'w');
fprintf(fid, 'system,type,index,value\n');
for i = 1:length(results)
    ev = results(i).eig_ct;
    for j = 1:length(ev)
        fprintf(fid, '%s,ct,%d,%.16e\n', results(i).name, j, ev(j));
    end
    ev = results(i).eig_dt;
    for j = 1:length(ev)
        fprintf(fid, '%s,dt,%d,%.16e\n', results(i).name, j, ev(j));
    end
end
fclose(fid);

% --- Condition number vs time (long format) ---
fid = fopen('obsv_cond_time.csv', 'w');
fprintf(fid, 'system,type,time,cond\n');
for i = 1:length(results)
    t = results(i).t_ct;   kap = results(i).kappa_ct;
    for j = 1:length(t)
        fprintf(fid, '%s,ct,%.8f,%.16e\n', results(i).name, t(j), kap(j));
    end
    t = results(i).t_dt;   kap = results(i).kappa_dt;
    for j = 1:length(t)
        fprintf(fid, '%s,dt,%.8f,%.16e\n', results(i).name, t(j), kap(j));
    end
end
fclose(fid);
fprintf('CSVs written: obsv_eigenvalues.csv, obsv_cond_time.csv\n');

%% =========================================================================
%  LOCAL FUNCTIONS
%% =========================================================================

function Wo = finite_horizon_gramian_ct(A, C, T, dt)
% Finite-horizon CT observability Gramian.
% Uses matrix-exponential propagation: expm(A*dt) computed once,
% then Phi = e^{At} stepped cheaply — avoids one expm call per step.
    n    = size(A, 1);
    CTC  = C' * C;
    eAdt = expm(A * dt);       % computed once
    Phi  = eye(n);             % Phi(0) = e^{A*0} = I
    Wo   = zeros(n);
    N    = round(T / dt);
    for k = 0:N                % integrate at t = 0, dt, 2dt, ..., T
        Wo  = Wo + (Phi' * CTC * Phi) * dt;
        Phi = eAdt * Phi;      % Phi -> e^{A*(k+1)*dt}
    end
end

function Wo = finite_horizon_gramian_dt(Ad, C, N)
% Finite-horizon DT observability Gramian: sum_{k=0}^{N-1} (Ad^k)' C'C Ad^k
    n   = size(Ad, 1);
    CTC = C' * C;
    Wo  = zeros(n);
    Ak  = eye(n);              % Ad^0
    for k = 0:N-1
        Wo = Wo + (Ak' * CTC * Ak);
        Ak = Ad * Ak;
    end
end

function [t_ct, kappa_ct, t_dt, kappa_dt] = cond_vs_time(A, C, Ad, Ts, T_stop, dt_sweep)
% Returns kappa(Wo(t)) = lambda_max / lambda_min for increasing horizon t.
%
% CT: uses expm propagation — one expm(A*dt_sweep) call.
% DT: running accumulation at each sample step.
    n   = size(A, 1);
    CTC = C' * C;

    % ----- Continuous time -----
    eAdt     = expm(A * dt_sweep);   % computed once
    Phi      = eye(n);
    Wo       = zeros(n);
    t_ct     = dt_sweep : dt_sweep : T_stop;
    kappa_ct = zeros(1, length(t_ct));
    for j = 1:length(t_ct)
        Wo  = Wo + Phi' * CTC * Phi * dt_sweep;
        Phi = eAdt * Phi;
        ev  = real(eig((Wo + Wo') / 2));
        kappa_ct(j) = max(ev) / min(ev);
    end

    % ----- Discrete time -----
    N_stop   = round(T_stop / Ts);
    t_dt     = (1:N_stop) * Ts;
    kappa_dt = zeros(1, N_stop);
    Wo       = zeros(n);
    Ak       = eye(n);               % Ad^0
    for k = 1:N_stop
        Wo = Wo + Ak' * CTC * Ak;
        Ak = Ad * Ak;
        ev = real(eig((Wo + Wo') / 2));
        kappa_dt(k) = max(ev) / min(ev);
    end
end