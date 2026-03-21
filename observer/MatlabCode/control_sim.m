%% control
% Cost matrices
Qlqr = diag([1e6,1e6,1e2, 1e1,1e1, 1e2,1e2,1e2, 1e2,1e2]);
Rlqr = 1e-0*eye(length(params.solenoids.r));

% Computing LQR estimate
Kcred = round(lqr(Ared,Bred,Qlqr,Rlqr),3); % Rounding can sometimes be dangerous!
Kred = dlqr(Ad,Bd,Qlqr,Rlqr);

% increasing order of our controller for controlling the real system
Kc = [Kcred(:,1:5), zeros(4,1), Kcred(:,6:end), zeros(4,1)];
K = [Kred(:,1:5), zeros(4,1), Kred(:,6:end), zeros(4,1)];

%writematrix(K, 'feedbackGain.csv');
%% simulation
% Initial conditions
x0 = xLp + [0.000 -0.00 0.04 0 0 0 zeros(1, 6)].';
tSpan = 0:dt:1;
N = length(tSpan);

% Observer initialization
clear observer_mex;
observer('init', 0);

% Preallocate arrays
x = zeros(length(x0), N);
x_est = zeros(length(x0), N);
NIS_values = zeros(1, N);
u = zeros(4, N);
y = zeros(3, N);
x(:,1) = x0;

% Simulation loop
for k = 1:N-1
    % Current time
    t_current = tSpan(k);
    
    % Get measurement at current time
    y(:,k) = h(x(:,k), zeros(4,1));  % Assuming feedthrough is compensated for

    
    % Estimation
    x_est(:,k) = x(:,k);
    [x_est(:,k), NIS_values(k)] = observer(u(:,k), y(:,k));
    
    % Compute control input using estimated state
    u(:,k) = -K * (x_est(:,k) - xLp) - uLp;
    
    % Simulate continuous plant from t_current to t_current+dt
    [~, x_cont] = ode15s(@(t,xf) f(xf, u(:,k)), [t_current t_current+dt], x(:,k));
    x(:,k+1) = x_cont(end,:).';
end

t = tSpan;

% plotting
figure(1);
clf; grid on; hold on; box on;

plot(t,x(1:3,:),'linewidth',2)

xlabel('t')
ylabel('x/y/z')
legend({'x','y','z'},'location','best')