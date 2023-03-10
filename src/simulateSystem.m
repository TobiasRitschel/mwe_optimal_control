% Solve dynamical parameter estimation problem
clc; clear all; close all; restoredefaultpath;

%% Initialize
% Right-hand side function
fun = @(t, x, u) -x + u;

% Measurement function
meas = @(t, x, u) x;

% Initial condition
x0 = 0;

% Manipulated input
u = 1;

% Time span
tspan = [0, 10];

% Options
opts = odeset();

% Simulate
[t, x] = ode45(fun, tspan, x0, opts, ...
    u);

% Visualize simulation
plot(t, x);

% Add more plots
hold on;

% Plot inputs
plot(t, repmat(u, size(t)));

% Stop adding plots
hold off;

% Axis limits
ylim([-0.1, 1.1]);