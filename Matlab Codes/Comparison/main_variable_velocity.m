%% Initialization
% In this code we have implemented the ACC problem comparing ICCBF (Input
% Constrained Control Barrier Function) and CLF-CBF (Control Lyapunov
% Function - Control Barrier Function) 

% utilities
clc
clear
close all

syms x1 x2 u v0 real 
x = [x1 x2]';   % state

% initial conditions of the state
x1d0 = 100;
x2d0 = 20;

% physical parameters
m = 1650;
f0 = 0.1;
f1 = 5;
f2 = 0.25;
Vref = 13.89;
g0 = 9.81;
vmax = 24;      % maximum velocity
dmax = 100;     % maximum distance considered
umax = 0.25;    % input contraints

Fr = f0 + f1*x2 + f2*x2^2;

% plot limits
limx = [0 dmax];
limy = [0 vmax];

% alpha's constants
k0 = 4;
k1 = 7;
k2 = 2;

% vector fields
f = [v0-x2; -Fr/m];
g = [0; g0];

% safety function
h = x1 - 1.8*x2;
Lfh = Lie(f, h, x);
Lgh = Lie(g, h, x);

% Control Lyapunov Function
V = (x2 - vmax)^2;
LfV = Lie(f, V, x);
LgV = Lie(g, V, x);

% Parameters for ode45
time = 100;
dt = 0.05;      % 50 Hz
tspan = 0 : dt : time;
ode_opt = odeset('MaxStep', 0.5);

% option for quadprog
qp_opt = optimset('Display', 'off');

% variable speed of the leading car as sinusoidal function
[vl, ~] = va_profiles(tspan);

%% computation of CBFs

b0 = h;
[db0_inf, db0_sup] = dcbf(x, f, g, b0, umax);

b1 = simplify(db0_inf + k0*b0);
[db1_inf, db1_sup] = dcbf(x, f, g, b1, umax);

b2 = simplify(db1_inf + k1*sqrt(b1));
[db2_inf, db2_sup] = dcbf(x, f, g, b2, umax);

% compute Lie derivatives of b2 
Lfb2 = simplify(Lie(f, b2, x));
Lgb2 = simplify(Lie(g, b2, x));

disp('CBFs computed correctly')

%% optimization problem ICCBF

% desired acceleration
ud = simplify((-LfV - 10*V)/(LgV));

% save the profile of x1, x2, u
x1_ic = zeros(1, max(size(tspan)));
x2_ic = zeros(1, max(size(tspan)));
u_ic = zeros(1, max(size(tspan)));

% state initialization
x1_ic(1) = x1d0;
x2_ic(1) = x2d0;

for i = 1: max(size(tspan))-1
    
    % numerical values for quadprog
    b2n = double(subs(b2, [x1 x2 v0]', [x1_ic(i) x2_ic(i) vl(i)]'));
    Lfb2n = double(subs(Lfb2, [x1 x2 v0]', [x1_ic(i) x2_ic(i) vl(i)]'));
    Lgb2n = double(subs(Lgb2, [x1 x2 v0]', [x1_ic(i) x2_ic(i) vl(i)]'));
    udn = double(subs(ud, x2, x2_ic(i)));
    
    % optimal control computed numerically
    u = quadprog(1, -udn, -Lgb2n, Lfb2n + k2*b2n, [], [], -umax, umax, [], qp_opt);
    
    % save the control
    u_ic(i+1) = u;
    
    fn = double(subs(f, [x2 v0]', [x2_ic(i) vl(i)]'));
    % system simulation
    [t_ic, x_ic] = ode45(@(t, x) sys(t, x, fn, g, u), [tspan(i) tspan(i+1)], [x1_ic(i) x2_ic(i)], ode_opt);
    
    % updating the initial state for the simulation
    idx = max(size(x_ic)); 
    x1_ic(i+1) = x_ic(idx, 1);
    x2_ic(i+1) = x_ic(idx, 2);
end

% evaluate the safety function
h_ic = x1_ic - 1.8*x2_ic;

disp('ICCBF method evaluated')


%% Optimization Problem CLF-CBF-QP

% save the profile of x1, x2, u
x1_cbf = zeros(1, max(size(tspan)));
x2_cbf = zeros(1, max(size(tspan)));
u_cbf = zeros(1, max(size(tspan)));

% state initialization
x1_cbf(1) = x1d0;
x2_cbf(1) = x2d0;

for i = 1: max(size(tspan))-1
    
    % numerical values for quadprog
    hn = double(subs(h, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    LfVn = double(subs(LfV, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    LgVn = double(subs(LgV, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    Lfhn = double(subs(Lfh, [x1 x2 v0]', [x1_cbf(i) x2_cbf(i) vl(i)]'));
    Lghn = double(subs(Lgh, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    Vn = double(subs(V, x2, x2_cbf(i)));
    
    % optimization parameters
    H = [1 0; 0 0.2];
    A = [LgVn -1; -Lghn 0];
    b = [-10*Vn-LfVn Lfhn+k2*hn]';
    
    % evaluation of the optimal control qp = [u; delta]
    qp = quadprog(H, [0 0]', A, b, [], [], [], [], [], qp_opt);
    u = qp(1);
    
    % clamp the control
    if u < -umax
        u = -umax;
    elseif u > umax
        u = umax;
    end
    
    % save the control
    u_cbf(i+1) = u;
    
    fn = double(subs(f, [x2 v0], [x2_cbf(i) vl(i)]));
    % system simulation
    [t_cbf, x_cbf] = ode45(@(t, x) sys(t, x, fn, g, u), [tspan(i) tspan(i+1)], [x1_cbf(i) x2_cbf(i)], ode_opt);
    
    % updating the initial state for simulation
    idx = max(size(x_cbf)); 
    x1_cbf(i+1) = x_cbf(idx, 1);
    x2_cbf(i+1) = x_cbf(idx, 2);
end

% evaluate the safety function
h_cbf = x1_cbf - 1.8*x2_cbf;

disp('CLF-CBF method evaluated')


%% Plots

figure('name', 'SAFETY PROFILE')


plot(tspan, h_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tspan, h_cbf, 'b--', 'linewidth', 2);
yline(0, 'k--');
title('Safety');
xlabel('Time [s]');
ylabel('h [m]');
[min_h, idc] = min(h_cbf);
text(tspan(idc) + 2, min_h, '\leftarrow unsafe')
legend('ICCBF-QP', 'CLF-CBF-QP', 'location', 'northeast');


figure('name', 'POSITION PROFILE')

plot(tspan, x1_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tspan, x1_cbf, 'b--', 'linewidth', 2);
yline(min([x1_ic x1_cbf]), 'k--');
title('Position profile');
xlabel('Time [s]');
ylabel('x_1 [m]');
legend('ICCBF-QP', 'CLF-CBF-QP', 'location', 'northeast');


figure('name', 'SPEED PROFILE')

plot(tspan, x2_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tspan, x2_cbf, 'b--', 'linewidth', 2);
plot(tspan, vl, 'r--', 'linewidth', 2);
yline(max([x2_ic x2_cbf]), 'k--');
yline(min([x2_ic x2_cbf]), 'k--');
title('Velocity');
xlabel('Time [s]');
ylabel('x_2 [m/s]');
legend('ICCBF-QP', 'CLF-CBF-QP', 'leading velocity', 'location', 'northeast');


figure('name', 'CONTROL PROFILE')

plot(tspan, u_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tspan, u_cbf, 'b--', 'linewidth', 2)
yline(umax, 'k--');
yline(-umax, 'k--');
ylim([-0.3 0.3]);
title('Control');
xlabel('Time [s]');
ylabel("u [g's]");
legend('ICCBF-QP', 'CLF-CBF-QP', 'location', 'northeast');

