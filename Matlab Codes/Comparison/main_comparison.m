%% Initialization
% In this code we have compared the results obtained for the ACC problem
% using ICCBF, CLF-CBF and CLF-CBF with Optimal CBF both for trapezoidal
% and constant velocity profiles

% utilities
clc
clear
close all

syms x1 x2 x3 u aL real 
x = [x1 x2 x3]';   % state

% initial conditions of the state
x1d0 = 150;     % distance between vehicles
x2d0 = 18;      % follower initial velocity
x3d0 = 10;      % leader initial velocity

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

% alpha's constants
k0 = 4;
k1 = 7;
k2 = 2;

Fr = f0 + f1*x2 + f2*x2^2;

% vector fields
f = [x3-x2; -Fr/m; aL];
g = [0; g0; 0];

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
dim = max(size(tspan));
ode_opt = odeset('MaxStep', 0.5);

% option for quadprog
qp_opt = optimset('Display', 'off');

% select velocity profile to be used
velocity_profile = 1;

disp(strcat('FLAG: velocity_profile= ', num2str(velocity_profile)))
disp('velocity_profile=0 -> constant velocity of leading vehicle with al=af')
disp('velocity_profile=1 -> trapezoidal velocity of leading vehicle with al>af')

if velocity_profile == 1
    [v_l, a_l] = va_profiles(tspan);
    al = max(abs(a_l));
else
    v_l = x3d0*ones([1, dim]);
    a_l = zeros(1, dim);
    al = 0.25;
end

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

%% ICCBF

% desired acceleration
ud = simplify((-LfV - 10*V)/(LgV));

% save the profile of x1, x2, u
x1_ic = zeros(1, dim);
x2_ic = zeros(1, dim);
x3_ic = zeros(1, dim);

u_ic = zeros(1, dim-1);

% state initialization
x1_ic(1) = x1d0;
x2_ic(1) = x2d0;
x3_ic(1) = x3d0;

for i = 1 : dim - 1
    
    % numerical values for quadprog
    b2n = double(subs(b2, [x1 x2 x3 aL]', [x1_ic(i) x2_ic(i) x3_ic(i) a_l(i)]'));
    Lfb2n = double(subs(Lfb2, [x1 x2 x3 aL]', [x1_ic(i) x2_ic(i) x3_ic(i) a_l(i)]'));
    Lgb2n = double(subs(Lgb2, [x1 x2 x3]', [x1_ic(i) x2_ic(i) x3_ic(i)]'));
    udn = double(subs(ud, x2, x2_ic(i)));
    
    % optimal control computed numerically
    u = quadprog(1, -udn, -Lgb2n, Lfb2n + k2*b2n, [], [], -umax, umax, [], qp_opt);
    
    % save the control
    u_ic(i) = u;
    
    fn = double(subs(f, [x2 x3 aL]', [x2_ic(i) x3_ic(i) a_l(i)]'));
    
    % system simulation
    [t_ic, x_ic] = ode45(@(t, x) sys(t, x, fn, g, u), [tspan(i) tspan(i+1)], [x1_ic(i) x2_ic(i) x3_ic(i)], ode_opt);
    
    % updating the initial state for the simulation
    idx = max(size(x_ic)); 
    x1_ic(i+1) = x_ic(idx, 1);
    x2_ic(i+1) = x_ic(idx, 2);
    x3_ic(i+1) = x_ic(idx, 3);
    
end

%evaluate the safety function
h_ic = x1_ic - 1.8*x2_ic;

disp('ICCBF method evaluated')


%% CLF-CBF-QP

% save the profile of x1, x2, u
x1_cbf = zeros(1, dim);
x2_cbf = zeros(1, dim);
x3_cbf = zeros(1, dim);

u_cbf = zeros(1, dim-1);

% state initialization
x1_cbf(1) = x1d0;
x2_cbf(1) = x2d0;
x3_cbf(1) = x3d0;

for i = 1 : dim - 1
    
    % numerical values for quadprog
    hn = double(subs(h, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    LfVn = double(subs(LfV, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    LgVn = double(subs(LgV, [x1 x2]', [x1_cbf(i) x2_cbf(i)]'));
    Lfhn = double(subs(Lfh, [x1 x2 x3]', [x1_cbf(i) x2_cbf(i) x3_cbf(i)]'));
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
    u_cbf(i) = u;
    
    fn = double(subs(f, [x2 x3 aL]', [x2_cbf(i) x3_cbf(i) a_l(i)]'));
    % system simulation
    [t_cbf, x_cbf] = ode45(@(t, x) sys(t, x, fn, g, u), [tspan(i) tspan(i+1)], [x1_cbf(i) x2_cbf(i) x3_cbf(i)], ode_opt);
    
    % updating the initial state for simulation
    idx = max(size(x_cbf)); 
    x1_cbf(i+1) = x_cbf(idx, 1);
    x2_cbf(i+1) = x_cbf(idx, 2);
    x3_cbf(i+1) = x_cbf(idx, 3);
    
end

% evaluate the safety function
h_cbf = x1_cbf - 1.8*x2_cbf;

disp('CLF-CBF method evaluated')

%% Optimal CBF used for CLF-CBF-QP

% input v.f. used in the optimal context
g = [0 1/m 0]';
LgV = Lie(g, V, x);

% save the profile of x1, x2, u
x1_cbf2 = zeros(1, dim);
x2_cbf2 = zeros(1, dim);
x3_cbf2 = zeros(1, dim);

u_cbf2 = zeros(1, dim-1);
h_cbf2 = zeros(1, dim-1);

% state initialization
x1_cbf2(1) = x1d0;
x2_cbf2(1) = x2d0;
x3_cbf2(1) = x3d0;

[h1, dh1, h2, dh2, h3, dh3] = ret_ZCBF_comp(x, f, g, af, al);

for i = 1: dim-1
    
    x_cbf0 = [x1_cbf2(i) x2_cbf2(i) x3_cbf2(i)]';

    % current values for velocity and acceleration of leader and follower
    vfn = x2_cbf2(i);
    vln = x3_cbf2(i);
    
    % choose the optimal ZCBF based on current values
    [h_opt, dh_opt] = opt_ZCBF(af, al, vfn, vln, g0, h1, h2, h3, dh1, dh2, dh3);
    
    % numerical values for quadprog
    Vn = double(subs(V, x, x_cbf0));
    LfVn = double(subs(LfV, x, x_cbf0));
    LgVn = double(subs(LgV, x, x_cbf0));
    hn = double(subs(h_opt, x, x_cbf0));
    Lfhn = double(subs(dh_opt(1), [x; aL], [x_cbf0; a_l(i)]));
    Lghn = double(subs(dh_opt(2), x, x_cbf0));
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
%     qp = quadprog(H, F, A, b, [], [], [-umax*m*g0 -Inf]', [umax*m*g0 +Inf]', [], qp_opt);
    qp = quadprog(H, F, A, b, [], [], [], [], [], qp_opt);
    u = qp(1);
    
    % save the control
    u_cbf2(i) = u/(m*g0);
    
    fn = double(subs(f, [x; aL], [x_cbf0; a_l(i)]));
    % system simulation
    [t_cbf2, x_cbf2] = ode45(@(t, x) sys(t, x, fn, g, u), [tspan(i) tspan(i+1)], [x1_cbf2(i) x2_cbf2(i) x3_cbf2(i)], ode_opt);
    
    % updating the state for simulation
    idx = max(size(x_cbf2)); 
    x1_cbf2(i+1) = x_cbf2(idx, 1);
    x2_cbf2(i+1) = x_cbf2(idx, 2);
    x3_cbf2(i+1) = x_cbf2(idx, 3);
    
    h_cbf2(i) = hn;
    
end

disp('Optimal CBF method evaluated')


%% Plots

tplot = linspace(0, time, dim-1);


figure('name', 'SAFETY PROFILE')

plot(tspan, h_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tspan, h_cbf, 'b--', 'linewidth', 2);
plot(tplot, h_cbf2, 'r-.', 'linewidth', 2);
yline(0, 'k--');
title('Safety');
xlabel('Time [s]');
ylabel('h [m]');
[min_h, idc] = min(h_cbf);
text(tspan(idc) + 2, min_h, '\leftarrow unsafe')
legend('ICCBF-QP', 'CLF-CBF-QP', 'Optimal CBF', 'location', 'northeast');


figure('name', 'POSITION PROFILE')

plot(tspan, x1_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tspan, x1_cbf, 'b--', 'linewidth', 2);
plot(tspan, x1_cbf2, 'r-.', 'linewidth', 2);
title('Position profile');
xlabel('Time [s]');
ylabel('x_1 [m]');
legend('ICCBF-QP', 'CLF-CBF-QP', 'Optimal CBF', 'location', 'northeast');


figure('name', 'SPEED PROFILE')

plot(tspan, v_l, 'c:', 'linewidth', 2); hold on; grid;
plot(tspan, x2_ic, 'g', 'linewidth', 2); 
plot(tspan, x2_cbf, 'b--', 'linewidth', 2);
plot(tspan, x2_cbf2, 'r-.', 'linewidth', 2);
title('Velocity');
xlabel('Time [s]');
ylabel('x_2 [m/s]');
legend('Velocity profile', 'ICCBF-QP', 'CLF-CBF-QP', 'Optimal CBF', 'location', 'northeast');


figure('name', 'CONTROL PROFILE')

plot(tplot, u_ic, 'g', 'linewidth', 2); hold on; grid;
plot(tplot, u_cbf, 'b--', 'linewidth', 2);
plot(tplot, u_cbf2, 'r-.', 'linewidth', 2);
yline(umax, 'k--');
yline(-umax, 'k--');
ylim([-0.3 0.3]);
title('Control');
xlabel('Time [s]');
ylabel("u [g's]");
legend('ICCBF-QP', 'CLF-CBF-QP', 'Optimal CBF', 'location', 'northeast');

