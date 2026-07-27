% main_pso.m
% Orchestrates the tuning of the SCARA hybrid controller using PSO.

clear; clc; close all;

% 1. Folder Structure Setup
addpath(fullfile(pwd, 'src'));

% 2. Workspace Initialization
fprintf('Initializing workspace parameters...\n');
init;

% 3. Search Space Definition
% Candidate vector mapping: [Kp_x, Kp_y, Kd_x, Kd_y, Kp_f, Ki_f, Kad_f]
num_vars = 7;

% Lower bounds
LB = [0.0, 0.0, 10.0, 0.0, 0.0, 0.5, 10.0];

% Upper bounds (maximum stiffness before mechanical resonance/chattering)
UB = [9660.78, 2526.61, 307.49, 80.42, 5.0, 10.0, 300.0];

% 4. Algorithm Configuration
fprintf('Configuring Particle Swarm Optimization...\n');

initial_seed = [4800.0, 1200.0, 150.0, 40.0, 2.5, 5.0, 150.0];

options = optimoptions('particleswarm', ...
    'SwarmSize', 30, ...
    'MaxIterations', 300, ...
    'Display', 'iter', ...
    'UseParallel', true, ...
    'PlotFcn', 'pswplotbestf', ...
    'InitialSwarmMatrix', initial_seed); 

% 5. Main Optimization Loop
fprintf('Starting Control Optimization...\n');

% Define the objective function as an anonymous function
objective_function = @(particle) cost_function_scara(particle);

% cache collision avoidance
cost_function_scara(LB);
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
fprintf('Kad_f = %.2f\n', best_gains(7));
fprintf('======================================================\n');

Simulink.fileGenControl('reset');
fprintf('Workers shut down.\n');

post_process_results(best_gains, best_cost, optimization_time, output);

rmpath(fullfile(pwd, 'src'));