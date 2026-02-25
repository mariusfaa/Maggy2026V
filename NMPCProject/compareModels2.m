addpath(genpath("."))

load_params(MaglevModel.Accurate);
paramsAccurate = params;

load_params(MaglevModel.Fast);
paramsFast = params;

[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(params,modelId);
xLp = [0,0,zEq(1)+0.001,zeros(1,9)]';
uLp = 0*ones(4,1);
fprintf("zEq=%.7f\n",zEq(1));

f_fast = @(x,u) maglevSystemDynamics(x,u,paramsFast,MaglevModel.Fast);
f_accurate = @(x,u) maglevSystemDynamics(x,u,paramsAccurate,MaglevModel.Accurate);
e = @(x,u) f_fast(x,u)-f_accurate(x,u);

r_range = 0.05;
N = 21;

x_range = linspace(-r_range, r_range, N);
y_range = linspace(-r_range, r_range, N);

E_pos = zeros(N, N);
E_lvel = zeros(N, N);
E_avel = zeros(N, N);
for i = 1:N
    for j = 1:N
        x_test = xLp;
        x_test(1) = x_range(i); % x position
        x_test(2) = y_range(j); % y position
        err = e(x_test, uLp);
        E_pos(j,i) = norm(err(1:5));
        E_lvel(j,i) = norm(err(7:9));
        E_avel(j,i) = norm(err(10:11));
    end
end

figure;
subplot(2,2,1);
imagesc(x_range, y_range, E_pos);
colorbar;
xlabel('x_1'); ylabel('x_2');
title('||f_{fast} - f_{accurate}|| for states x_1 to x_5');
axis xy;

subplot(2,2,2);
imagesc(x_range, y_range, E_lvel);
colorbar;
xlabel('x_1'); ylabel('x_2');
title('||f_{fast} - f_{accurate}|| for states x_7 to x_9');
axis xy;

subplot(2,2,3);
imagesc(x_range, y_range, E_avel);
colorbar;
xlabel('x_1'); ylabel('x_2');
title('||f_{fast} - f_{accurate}|| for states x_{10} to x_{11}');
axis xy;