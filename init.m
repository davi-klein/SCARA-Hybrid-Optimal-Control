% =========================================================================
% """
% Initialization module for the Hybrid Position/Force SCARA Robot Simulation.
%
% This script initializes the MATLAB workspace with all necessary physical,
% environmental, and control parameters required for the Simulink model. 
% The planar kinematic and dynamic parameters (Links 1 and 2) are strictly 
% adapted from Massaro et al. (2023), "An Optimal Control Approach to the 
% Minimum-Time Trajectory Planning of Robotic Manipulators", Table 1.
%
% The 3rd degree of freedom (prismatic Z-axis) and contact dynamics are 
% defined to enable the Raibert-Craig hybrid control architecture.
%
% Attributes:
%     L (array): Link lengths in meters [L1, L2].
%     m (array): Link and joint masses in kg [m1, m2, m3].
%     I (array): Moments of inertia in kg.m^2 [I1, I2].
%     cm (array): Center of mass distances from joints in meters [b1, b2].
%     m_load (float): Payload mass at the end-effector in kg.
%     friction_viscous (array): Viscous friction coefficients [B1, B2, B3].
%     friction_coulomb (array): Coulomb friction coefficients [Fc1, Fc2, Fc3].
%     env_stiffness (float): Kelvin-Voigt environment stiffness (N/m).
%     env_damping (float): Kelvin-Voigt environment damping (N.s/m).
%     z_surface (float): Vertical position of the contact surface (m).
% """
% =========================================================================
clear; clc; close all;

%% 1. Physical & Geometric Parameters (Adapted from Massaro et al., 2023)
% Link lengths (meters)
L1 = 0.4; 
L2 = 0.25;

% Masses (kg)
m1 = 29.58;
m2 = 15.0;
m3 = 5.0;      % Prismatic Z-axis mass (Added for SCARA architecture)

% Center of mass locations (meters) - assumed at the middle of the links
b1 = L1 / 2;
b2 = L2 / 2;

% Moments of inertia with respect to the center of mass (kg.m^2)
I1 = 0.417;
I2 = 0.206;

% Mass of the payload (kg)
m_load = 6.0; 

%% 2. Friction Parameters (Viscous and Coulomb)
% Viscous friction (N.m.s/rad for rotational; N.s/m for prismatic)
friction_viscous = [0.5; 0.5; 0.1]; 

% Coulomb friction (N.m for rotational; N for prismatic)
friction_coulomb = [0.2; 0.2; 0.05];

%% 3. Environment Parameters (Kelvin-Voigt Contact Model)
env_stiffness = 15000.0; % Surface stiffness (N/m)
env_damping = 150.0;     % Surface damping (N.s/m)
z_surface = 0.0;         % Location of the rigid surface (m)

%% 4. Initial Conditions & Gravity
g = 9.81; % Gravity acceleration (m/s^2)

% Initial joint positions [q1 (rad); q2 (rad); d3 (m)]
q0 = [pi/4; pi/4; 0.10]; 

% Initial joint velocities [dq1; dq2; dd3]
dq0 = [0.0; 0.0; 0.0];

%% 5. Controller Gains (Initial Tuning for PSO later)
% Position Control Gains (XY Plane)
Kp_p = diag([5000.0, 5000.0, 0.0]);
Kd_p = diag([500.0, 500.0, 0.0]);

% Force Control Gains (Z Axis)
Kp_f = diag([0.0, 0.0, 1.5]);
Ki_f = diag([0.0, 0.0, 5.0]);

disp('Workspace successfully initialized. Ready to run the Simulink model.');