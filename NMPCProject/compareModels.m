load_params(MaglevModel.Fast);
paramsFast = params;

load_params(MaglevModel.Accurate);
paramsAccurate = params;
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,modelId);
xLp = [0,0,zEq(1),zeros(1,9)]';
fprintf("zEq=%.7f\n",zEq(1));

x0 = xLp + [0 0.00025 0.0045 0 deg2rad(2) 0 zeros(1, 6)].';
u = 0.7*ones(4,1);
dt = 0.001;
tSpan = 0:dt:0.1;

f_fast = @(x,u) maglevSystemDynamics(x,u,paramsFast,MaglevModel.Fast);
f_accurate = @(x,u) maglevSystemDynamics(x,u,paramsAccurate,MaglevModel.Accurate);

[tf,xf] = ode15s(@(t,x) f_fast(x,u), tSpan, x0);
[ta,xa] = ode15s(@(t,x) f_accurate(x,u), tSpan, x0);

xf = xf'; xa = xa';

% Plot
figure(8);
title("Magnet position");
clf; grid on; hold on; box on;
subplot(4,1,1);
plot(tSpan,xf(1:3,:),'linewidth',2)
ylabel('x/y/z fast')
ylim([-0.05,0.05])
legend({'x','y','z'},'location','best')

subplot(4,1,2);
plot(tSpan,xa(1:3,:),'linewidth',2)
xlabel('t')
ylabel('x/y/z accurate')
ylim([-0.05,0.05])
legend({'x','y','z'},'location','best')

subplot(4,1,3);
plot(tSpan,xf(4:6,:),'linewidth',2)
ylabel('x/y/z fast')
ylim([-0.05,0.05])
legend({'x','y','z'},'location','best')

subplot(4,1,4);
plot(tSpan,xa(4:6,:),'linewidth',2)
xlabel('t')
ylabel('x/y/z accurate')
ylim([-0.05,0.05])
legend({'x','y','z'},'location','best')

figure(9);
clf; grid on; hold on; box on;
I = 1:12;
loge = log10(xf(I,:)-xa(I,:));
plot(tSpan,loge,'linewidth',2)
title("errors")
ylabel('fast')
% ylim([-0.05,0.05])
legend({'x','y','z','\theta', '\phi','\rho','vx','vy','vz','\omega_\theta', '\omega_\phi','\omega_\rho'})