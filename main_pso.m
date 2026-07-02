% main_pso.m
% Orchestrates the tuning of the SCARA hybrid controller using PSO.

clear; clc; close all;

% 1. Folder Structure Setup
addpath(fullfile(pwd, 'src'));

% 2. Workspace Initialization
% Loads physical parameters, initial states, and base matrices from the src folder
fprintf('Initializing workspace parameters...\n');
init; 

% 3. Search Space Definition
% Candidate vector mapping: [Kp_x, Kp_y, Kd_x, Kd_y, Kp_f, Ki_f]
num_vars = 6;

% Lower bounds (minimum stiffness required for basic stability)
LB = [0.0, 0.0,  0.0,  0.0,  0.0,  0.0];

% Upper bounds (maximum stiffness before mechanical resonance/chattering)
UB = [20000.0, 20000.0, 2000.0, 2000.0, 10.0, 50.0];

% 4. Algorithm Configuration
fprintf('Configuring Particle Swarm Optimization...\n');

% Set PSO options. Enabling 'UseParallel' automatically distributes the 
% Simulink evaluations across available CPU cores. 'pswplotbestf' provides
% a live updating chart of the convergence.
options = optimoptions('particleswarm', ...
    'SwarmSize', 50, ...
    'MaxIterations', 100, ...
    'Display', 'iter', ...
    'UseParallel', false, ...
    'PlotFcn', 'pswplotbestf'); 

% 5. Main Optimization Loop
fprintf('Starting Control Optimization...\n');

% Define the objective function as an anonymous function
objective_function = @(particle) cost_function_scara(particle);

tic;
% Execute the built-in PSO algorithm
[best_gains, best_cost, exitflag, output] = particleswarm(objective_function, num_vars, LB, UB, options);
optimization_time = toc;

% 6. Results
fprintf('\n================ OPTIMIZATION RESULTS ================\n');
fprintf('Optimization completed in %.2f seconds.\n', optimization_time);
fprintf('Total Iterations: %d\n', output.iterations);
fprintf('Total Function Evaluations: %d\n', output.funccount);
fprintf('Final Best Cost (ITAE + Penalties): %.6f\n', best_cost);
fprintf('\nOptimal Gains:\n');
fprintf('Kp_x = %.2f\n', best_gains(1));
fprintf('Kp_y = %.2f\n', best_gains(2));
fprintf('Kd_x = %.2f\n', best_gains(3));
fprintf('Kd_y = %.2f\n', best_gains(4));
fprintf('Kp_f = %.2f\n', best_gains(5));
fprintf('Ki_f = %.2f\n', best_gains(6));
fprintf('======================================================\n');

rmpath(fullfile(pwd, 'src'));