%% Test Script: Compare ellipke_casadi vs MATLAB Ground Truth
m_test = linspace(0, 1-1e-5, 1000);
[K_true, E_true] = ellipke(m_test);

% Prepare CasADi function for testing
import casadi.*
m_sym = SX.sym('m');
[K_sym, E_sym] = ellipke_casadi(m_sym);
f_test = Function('f_test', {m_sym}, {K_sym, E_sym});

% Evaluate
[K_calc, E_calc] = f_test(m_test);
K_calc = full(K_calc);
E_calc = full(E_calc);

% Plot Results
figure('Name', 'Elliptic Integral Accuracy');
subplot(2,1,1);
plot(m_test, K_true, 'k', 'LineWidth', 2); hold on;
plot(m_test, K_calc, 'r--', 'LineWidth', 1.5);
title('K(m) Comparison'); legend('MATLAB', 'CasADi'); grid on;

subplot(2,1,2);
semilogy(m_test, abs(K_true - K_calc), 'r');
title('Absolute Error in K(m)'); xlabel('m (k^2)'); grid on;

fprintf('Max Error in K: %e\n', max(abs(K_true - K_calc)));
fprintf('Max Error in E: %e\n', max(abs(E_true - E_calc)));
fprintf('Value at m=0 (should be pi/2): %f\n', K_calc(1));