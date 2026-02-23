


% We want to simplify our function

% points per var
N = 101;

r = 0.1;
x = linspace(-r,r,N);
y = linspace(-r,r,N);
z = linspace(-r,r,N);
I = params.permanent.J/params.physical.mu0*params.permanent.l(1);
%ub = 1.5;
%I = linspace(-ub,ub,N);


% Fixed center values
x0 = 0; y0 = 0; z0 = 0;

% --- Vary x, fix y=0, z=0 ---
bx_x = zeros(1,N); by_x = zeros(1,N); bz_x = zeros(1,N);
for i = 1:N
    [bx_x(i),by_x(i),bz_x(i)] = computeFieldCircularCurrentSheetCartesian(...
        x(i), y0, z0, params.permanent.r(1), params.permanent.l(1), I, params.physical.mu0);
end

% --- Vary y, fix x=0, z=0 ---
bx_y = zeros(1,N); by_y = zeros(1,N); bz_y = zeros(1,N);
for i = 1:N
    [bx_y(i),by_y(i),bz_y(i)] = computeFieldCircularCurrentSheetCartesian(...
        x0, y(i), z0, params.permanent.r(1), params.permanent.l(1), I, params.physical.mu0);
end

% --- Vary z, fix x=0, y=0 ---
bx_z = ones(1,N); by_z = zeros(1,N); bz_z = zeros(1,N);
for i = 1:N
    [bx_z(i),by_z(i),bz_z(i)] = computeFieldCircularCurrentSheetCartesian(...
        x0, y0, z(i), params.permanent.r(1), params.permanent.l(1), I, params.physical.mu0);
end

% --- Plots ---
figure;

subplot(3,3,1); plot(x, bx_x); xlabel('x'); ylabel('Bx'); title('Bx vs x');
subplot(3,3,2); plot(x, by_x); xlabel('x'); ylabel('By'); title('By vs x');
subplot(3,3,3); plot(x, bz_x); xlabel('x'); ylabel('Bz'); title('Bz vs x');

subplot(3,3,4); plot(y, bx_y); xlabel('y'); ylabel('Bx'); title('Bx vs y');
subplot(3,3,5); plot(y, by_y); xlabel('y'); ylabel('By'); title('By vs y');
subplot(3,3,6); plot(y, bz_y); xlabel('y'); ylabel('Bz'); title('Bz vs y');

subplot(3,3,7); plot(z, bx_z); xlabel('z'); ylabel('Bx'); title('Bx vs z');
subplot(3,3,8); plot(z, by_z); xlabel('z'); ylabel('By'); title('By vs z');
subplot(3,3,9); plot(z, bz_z); xlabel('z'); ylabel('Bz'); title('Bz vs z');

sgtitle('B-field components: one input varied at a time');