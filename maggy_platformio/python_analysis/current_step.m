s = tf("s");
figure(1);
clf; grid on;
K=1;
tau = 0.0003;
sys = K/(1+tau*s);
step(sys,linspace(0,0.002,100));

x0 = 0.2;
x = 1.2;

f = @(t) x0 + (x-x0)*exp(-t/tau);