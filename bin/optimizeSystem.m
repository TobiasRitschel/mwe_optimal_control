%% Solve a basic dynamic optimization problem
% Clear command window
clc;

% Clear variables
clear all; %#ok

% Close figures
close all;

% Remove added paths
restoredefaultpath;

%% Formatting
% Font size
fs = 12;

% Line width
lw = 3;

% Marker size
ms = 12;

% Set default font size
set(groot, 'DefaultAxesFontSize',   fs);

% Set default line widths
set(groot, 'DefaultLineLineWidth',  lw);
set(groot, 'DefaultStairLineWidth', lw);
set(groot, 'DefaultStemLineWidth',  lw);

% Set default marker size
set(groot, 'DefaultLineMarkerSize', ms);

% Set default figure size
set(groot, 'DefaultFigureUnits', 'Inches', 'DefaultFigurePosition', [1, 1, 8, 6]);

% Set renderer
% (Otherwise, eps figures can become pixelated in Latex.)
% (Save as pdf if you have shaded areas in your figure, e.g., because
% you're using fill.)
set(groot, 'DefaultFigureRenderer', 'Painters');

%% Initialize
% Reference
yref = 0.5;

% Right-hand side function
fun = @(t, x, u) -x.^3 + u;

% Measurement function
output = @(t, x, u) x;

% Stage cost
stageCost = @(t, x, u, y) (y - yref).^2;

%% Simulate
% Number of control intervals
N = 10;

% Initial condition
x0 = 0;

% Manipulated input
ubar = linspace(1, 0.5, N);

% Time span
tspan = linspace(0, 10, N+1);

% Options
odeopts = odeset();

% Open-loop simulation
[T, X] = openLoopSimulation(ubar, x0, tspan, fun, odeopts);

% Create figure
figure(1);

% Visualize simulation
plot(T, X);

% Add more plots
hold on;

% Plot inputs
stairs(tspan, ubar([1:end, end]));

% Stop adding plots
hold off;

% Axis limits
ylim([-0.1, 1.1]);

% Title
title('Simulation');

%% Optimize
% Inputs for fmincon
A       = [];
B       = [];
Aeq     = [];
Beq     = [];
nonlcon = [];

ub =  ones(1, N);
lb = zeros(1, N);

% Options for fmincon
optopts = optimset(    ...
    'Display', 'iter', ...
    'TolFun', 1e-8);

% Initial guess of optimal control
u0 = 0.1*ones(1, N);

% Use fmincon to solve the numerical optimization problem
uopt = fmincon(@evaluateObjectiveFunction, u0, A, B, Aeq, Beq, lb, ub, nonlcon, optopts, ...
    x0, tspan, fun, output, stageCost, odeopts);

%% Simulate
% Open-loop simulation
[Topt, Xopt] = openLoopSimulation(uopt, x0, tspan, fun, odeopts);


% Create figure
figure(2);

% Plot setpoint
plot(tspan, repmat(yref, size(tspan)), '--k');

% Reset color ordering
set(gca, 'ColorOrderIndex', 1);

% Add more plots
hold on;

% Plot inputs
stairs(tspan, uopt([1:end, end]));

% Visualize simulation
plot(Topt, Xopt);

% Stop adding plots
hold off;

% Axis limits
ylim([-0.1, 1.1]);

% Title
title('Optimal control');

function J = evaluateObjectiveFunction(ubar, x0, tspan, fun, outp, stageCost, odeopts)

% Number of control intervals
N = size(ubar, 2);

% Initialize objective function value
J = 0;

for k = 1:N
    % Manipulated inputs in current control interval
    uk = ubar(:, k);

    % Simulate
    [t, x] = ode45(fun, tspan([k, k+1]), x0, odeopts, ...
        uk);

    % Evaluate outputs
    y = outp(t, x, uk);

    % Evaluate stage cost
    Phi = stageCost(t, x, uk, y);

    % Evaluate objective function
    J = J + sum(Phi(2:end).*diff(t(:)));

    % Update initial states
    x0 = x(end, :);
end
end

function [T, X] = openLoopSimulation(ubar, x0, tspan, fun, odeopts)

% Number of control intervals
N = size(ubar, 2);

% Initialize
T = [];
X = [];

% Initial states
xk = x0;

for k = 1:N
    % Manipulated input in current control interval
    uk = ubar(:, k);

    % Simulate
    [t, x] = ode45(fun, tspan([k, k+1]), xk, odeopts, ...
        uk);

    % Update initial states
    xk = x(end, :);

    % Store results
    T = [T; t]; %#ok
    X = [X; x]; %#ok
end
end
