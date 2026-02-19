x = linspace(0, 2*pi, 1000);
y = sin(x);

z1 = trapz(x, y);
z2 = trapz_casadi(x, y);

fprintf('trapz:        %.6f\n', z1);
fprintf('trapz_casadi: %.6f\n', z2);
fprintf('error:        %.2e\n', abs(z1-z2));
y2 = cos(x);
z3 = trapz(x, y2);
z4 = trapz_casadi(x, y2);
fprintf('\ntrapz cos:        %.6f\n', z3);
fprintf('trapz_casadi cos: %.6f\n', z4);
fprintf('error:            %.2e\n', abs(z3-z4));

x = linspace(0, 2*pi, 1000);
functions = {@sin, @cos, @(x) x.^2, @(x) exp(-x)};
names = {'sin', 'cos', 'x^2', 'exp(-x)'};

figure;
for i = 1:4
    y = functions{i}(x);
    z1 = trapz(x, y);
    z2 = trapz_casadi(x, y);
    
    subplot(2,2,i);
    plot(x, y);
    hold on;
    title(sprintf('%s | trapz=%.4f, casadi=%.4f, err=%.2e', ...
        names{i}, z1, z2, abs(z1-z2)));
    grid on;
    xlabel('x');
end
sgtitle('trapz vs trapz\_casadi comparison');