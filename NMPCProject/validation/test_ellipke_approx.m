M = linspace(0.01,0.99,100);
[K,E] = ellipke(M);
[K2,E2] = ellipke_casadi(M);

figure;
subplot(2,1,1);
plot(M,K,M,E,M,K2,'--',M,E2,'--');
grid on;
xlabel('M');
title('Complete Elliptic Integrals');
legend('K','E','K approx','E approx');

subplot(2,1,2);
plot(M, K-K2, M, E-E2);
grid on;
xlabel('M');
title('Approximation Error');
legend('K error','E error');