import casadi.*

parameters_maggy_V4;
correctionFactorFast = computeSolenoidRadiusCorrectionFactor(params,'fast');
paramsFast = params;
paramsFast.solenoids.r = correctionFactorFast*paramsFast.solenoids.r;
[zEq, zEqInv, dzEq, dzEqInv] = computeSystemEquilibria(paramsFast,'fast');

% Standard implementation of dynamics
f_std = @(x,u) maglevSystemDynamics(x,u,paramsFast,'fast');

% CasADi implementation of dynamics
x = SX.sym('x',12,1);
u = SX.sym('u', 4, 1);
f_expl = maglevSystemDynamicsCasADi(x, u, paramsFast);
f_cas = casadi.Function('f', {x, u}, {f_expl});

% Initial value/ testing values
test_x0 = [0,0,zEq(1),zeros(1,9)]' + [0.001 -0.003 0.04 pi/10 0 0 zeros(1, 6)].';
test_u0 = zeros(4,1);

% Evaluate CasADi implementation
f_test_cas = full(f_cas(test_x0, test_u0));
disp('Dynamics at x0, u=0:');
disp(f_test_cas);

% Evaluate standard numerical implementation
f_test_std = full(f_std(test_x0, test_u0));
disp('Dynamics at x0, u=0:');
disp(f_test_std);

% Discrepency between implementations
discrepancy = abs(f_test_std) - abs(f_test_cas);
disp('Discrepancy between standard and CasADi implementations:');
disp(discrepancy);