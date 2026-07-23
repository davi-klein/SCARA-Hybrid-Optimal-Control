% =========================================================================
% """
% Initialization module for the Hybrid Position/Force SCARA Robot Simulation.
%
% This script initializes the MATLAB workspace with all necessary physical,
% environmental, and control parameters required for the Simulink model. The
% parameters are based on the Omron Adept 550 SCARA robot, specifically 
% tailored for planar polishing tasks using a Mirka AIROS 350CV Ø77mm tool.
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

% 1. Physical & Geometric Parameters (Adapted from Boschetti & Sinico, 2025)
% Link lengths (meters) 
L1 = 0.300; % 300 mm
L2 = 0.250; % 250 mm

% Masses (kg)
m1 = 25.56;    % Link 1 estimated mass
m2 = 13.21;    % Link 2 estimated mass
m3 = 1.2254;   % Prismatic Z-axis (ball screw spline) mass empirically identified

% Center of mass locations (meters) - assumed at the middle of the links
b1 = L1 / 2;
b2 = L2 / 2;

% Moments of inertia with respect to the center of mass (kg.m^2)
I1 = 0.60;     % Derived from identifiable parameters
I2 = 0.22;     % Derived from identifiable parameters

% Mass of the payload (kg) - Mirka AIROS 350CV Ø77mm Polishing Tool
m_load = 1.1; 

% 2. Friction Parameters (Viscous and Coulomb) 
% Data extracted from standard identification procedure (Table 4)
% Viscous friction (N.m.s/rad for rotational; N.s/m for prismatic)
friction_viscous = [17.3010; 2.3557; 0.0033];

% Coulomb friction (N.m for rotational; N for prismatic)
friction_coulomb = [13.7267; 4.6119; 0.1319];

% 3. Environment Parameters (Kelvin-Voigt Contact Model)
env_stiffness = 15000.0; % Surface stiffness (N/m)
env_damping = 150.0;     % Surface damping (N.s/m)
z_surface = 0.0;         % Location of the rigid surface (m)

% 4. Initial Conditions & Gravity
g = 9.807; % Gravity acceleration (m/s^2)

% Initial joint positions [q1 (rad); q2 (rad); d3 (m)]
q0 = [pi/4; pi/4; 0.0]; 

% Initial joint velocities [dq1; dq2; dd3]
dq0 = [0.0; 0.0; 0.0];

% 5. Controller Gains (Initial Tuning for PSO later)
% Position Control Gains (XY Plane)
Kp_p = diag([50.0, 50.0, 0.0]);
Kd_p = diag([25.0, 25.0, 0.0]);

% Force Control Gains (Z Axis)
Kp_f = diag([0.0, 0.0, 2.0]);
Ki_f = diag([0.0, 0.0, 0.0]);

% Active Damping for Z-axis impact (Raibert-Craig addition)
Kd_f = 25.0;

% Derivative filter
N = 50;

disp('Workspace successfully initialized. Ready to run the Simulink model.');