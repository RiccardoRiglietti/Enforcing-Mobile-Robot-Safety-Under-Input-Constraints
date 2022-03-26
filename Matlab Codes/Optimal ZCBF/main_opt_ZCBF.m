%% Initialization
% In this code we have implemented the ACC problem using the optimal ZCBF 
% (Zeoring Control Barrier Function) considering a trapezoidal profile for
% the speed of the leading vehicle. 

% utilities
clc
clear
close all

syms x1 x2 x3 u aL vl real 
x = [x1 x2 x3]';   % state

% initial conditions of the state
x1d0 = 18;
x2d0 = 10;
x3d0 = 150;
xd0 = [x1d0 x2d0 x3d0]';    % initial conditions

% physical parameters
m = 1650;
f0 = 0.1;
f1 = 5;
f2 = 0.25;
g0 = 9.81;
vmax = 22;      % maximum velocity
dmax = 100;     % maximum distance considered
umax = 0.25;    % input contraints
psc = 1e2;
af = 0.25;

Fr = f0 + f1*x1 + f2*x1^2;

% Vector fields defined as in the paper:
% x1 = vf x2 = vl and x3 = D

f = [-Fr/m; aL; x2-x1];
g = [1/m; 0; 0];

% Control Lyapunov Function
V = (x1 - vmax)^2;
LfV = Lie(f, V, x);
LgV = Lie(g, V, x);

% Parameters for ode45
time = 100;
dt = 0.05;      % 50 Hz
tspan = 0 : dt : time;
ode_opt = odeset('MaxStep', 0.5);

% Option for quadprog
qp_opt = optimset('Display', 'off');

% Speed and acceleration profile of the leading car
[v_l, a_l] = va_profiles(tspan);
al = max(abs(a_l));

%% Optimization Problem CLF-CBF-QP

dim = max(size(tspan));

% save the profile of x1, x2, u
x1_cbf = zeros(1, dim);
x2_cbf = zeros(1, dim);
x3_cbf = zeros(1, dim);

u_cbf = zeros(1, dim-1);
h_cbf = zeros(1, dim-1);

% state initialization
x1_cbf(1) = x1d0;
x2_cbf(1) = x2d0;
x3_cbf(1) = x3d0;

[h1, dh1, h2, dh2, h3, dh3] = ret_ZCBF(x, f, g, af, al);

for i = 1: max(size(tspan))-1
    
    x_cbf0 = [x1_cbf(i) x2_cbf(i) x3_cbf(i)]';

    % current values for velocity and acceleration of leader and follower
    vfn = x1_cbf(i);
    vln = x2_cbf(i);
    
    % choose the optimal ZCBF based on current values
    [h, dh] = opt_ZCBF(af, al, vfn, vln, g0, h1, h2, h3, dh1, dh2, dh3);
    
    % numerical values for quadprog
    Vn = double(subs(V, x, x_cbf0));
    LfVn = double(subs(LfV, x, x_cbf0));
    LgVn = double(subs(LgV, x, x_cbf0));
    hn = double(subs(h, x, x_cbf0));
    Lfhn = double(subs(dh(1), [x; aL], [x_cbf0; a_l(i)]));
    Lghn = double(subs(dh(2), x, x_cbf0));
    Frn = double(subs(Fr, x, x_cbf0));
    
    if ~isreal(hn) || ~isreal(Lghn) || ~isreal(Lfhn)
        disp(num2str(i))
    end
    
    % optimization parameters
    H = 2*[1/m^2 0; 0 psc];
    F = -2*[Frn/m^2; 0];
    A = [LgVn -1; -Lghn 0; 1 0; -1 0];
    b = [-10*Vn-LfVn; Lfhn+hn; umax*m*g0; umax*m*g0];
    
    % evaluation of the optimal control qp = [u; delta]
    qp = quadprog(H, F, A, b, [], [], [-umax*m*g0 -Inf]', [umax*m*g0 +Inf]', [], qp_opt);
    u = qp(1);
    
    % save the control
    u_cbf(i) = u;
    
    fn = double(subs(f, [x; aL], [x_cbf0; a_l(i)]));
    % system simulation
    [t_cbf, x_cbf] = ode45(@(t, x) sys(t, x, fn, g, u), [tspan(i) tspan(i+1)], [x1_cbf(i) x2_cbf(i) x3_cbf(i)], ode_opt);
    
    % updating the state for simulation
    idx = max(size(x_cbf)); 
    x1_cbf(i+1) = x_cbf(idx, 1);
    x2_cbf(i+1) = x_cbf(idx, 2);
    x3_cbf(i+1) = x_cbf(idx, 3);
    
    h_cbf(i) = hn;
    
end

disp('CLF-CBF method evaluated')

%% plots

tplot = linspace(0, time, dim-1);

figure('name', 'SAFETY PROFILE')

plot(tplot, h_cbf, 'b', 'linewidth', 2);hold on; grid;
yline(0, 'k--');
title('Safety');
xlabel('Time [s]');
ylabel('h [m]');
xlim([0 time]);
legend( 'CLF-CBF-QP', 'location', 'northeast');


figure('name', 'SPEED PROFILES')

plot(tspan, x1_cbf, 'b', 'linewidth', 2); hold on; grid;
plot(tspan, x2_cbf, 'r--', 'linewidth', 2);
title('Velocity');
xlabel('Time [s]');
ylabel('x_1, x_2 [m/s]');
legend('following', 'leading', 'location', 'northeast');


figure('name', 'CONTROL PROFILE')

plot(tplot, u_cbf/(m*g0), 'b', 'linewidth', 2); hold on; grid;
yline(umax, 'k--');
yline(-umax, 'k--');
title('Control');
xlabel('Time [s]');
ylabel("u [g's]");
legend('CLF-CBF-QP', 'location', 'northeast');


figure('name', 'POSITION PROFILE')

plot(tspan, x3_cbf, 'b', 'linewidth', 2); hold on; grid;
title('Position');
xlabel('Time [s]');
ylabel("d [m]");
legend('CLF-CBF-QP', 'location', 'northeast');