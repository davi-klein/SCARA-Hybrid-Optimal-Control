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
%     pi_params (array): Lumped dynamic coefficients [pi_1, pi_2, pi_3, pi_4, pi_5].
%     motor_inertia (array): Reflected motor inertias [Im2_eq, Im3_eq].
%     m_load (float): Payload mass at the end-effector in kg.
%     friction_viscous (array): Viscous friction coefficients [B1, B2, B3].
%     friction_coulomb (array): Coulomb friction coefficients [Fc1, Fc2, Fc3].
%     env_stiffness (float): Kelvin-Voigt environment stiffness (N/m).
%     env_damping (float): Kelvin-Voigt environment damping (N.s/m).
%     z_surface (float): Vertical position of the contact surface (m).
% """
% =========================================================================

% 1. Physical & Geometric Parameters (Adapted from Boschetti and Sinico, 2025)
% Link lengths (meters) 
L1 = 0.300; % 300 mm
L2 = 0.250; % 250 mm

% Lumped Dynamic Coefficients (Table 7 - Proposed Identification after optimization)
pi_1 = 2.6001; % m2*a1^2 + Im1_eq + J1_zz
pi_2 = 1.6511; % m2*c2x
pi_3 = 0.4298; % J2_zz + J3_zz
pi_4 = 1.7533; % m3 (Z-axis moving mass without payload)
pi_5 = 0.1531; % J4_zz (Adds to J2_zz since joint 4 is locked)

% Reflected actuator inertias (Table 7)
Im2_eq = 0.0425;
Im3_eq = 1.4e-5;

% Mass of the payload (kg) - Mirka AIROS 350CV Ø77mm Polishing Tool
m_load = 1.1; 

% 2. Friction Parameters (Viscous and Coulomb) 
% Data extracted from standard identification procedure (Table 4)
% Viscous friction (N.m.s/rad for rotational; N.s/m for prismatic)
friction_viscous = [17.3010; 2.3557; 0.0033];

% Coulomb friction (N.m for rotational; N for prismatic)
friction_coulomb = [13.7267; 4.6119; 0.1319];

fric_params_ctrl = [friction_viscous; friction_coulomb] * 0.85; % Underestimation for control robustness evaluation

% 3. Environment Parameters (Kelvin-Voigt Contact Model)
env_stiffness = 15000.0; % Surface stiffness (N/m)
env_damping = 150.0;     % Surface damping (N.s/m)
z_surface = 0.0;         % Location of the rigid surface (m)

% 4. Initial Conditions & Gravity
g = 9.807; % Gravity acceleration (m/s^2)

% Calculate Exact Initial Joint Positions via Inverse Kinematics
% Starting Cartesian position based on trajectory generator at t=0
x_start = -0.100;
y_start = 0.350 + 0.040; % y_center + R*cos(0)

% Inverse Kinematics for SCARA (Elbow Right configuration)
r_sq = x_start^2 + y_start^2;
cos_q2 = (r_sq - L1^2 - L2^2) / (2 * L1 * L2);
sin_q2 = sqrt(1 - cos_q2^2);

q2_0 = atan2(sin_q2, cos_q2);
q1_0 = atan2(y_start, x_start) - atan2(L2 * sin_q2, L1 + L2 * cos_q2);

% Initial joint positions [q1 (rad); q2 (rad); d3 (m)]
q0 = [q1_0; q2_0; 0.005];

% Initial joint velocities [dq1; dq2; dd3]
dq0 = [0.0; 0.0; 0.0];

% 5. Controller Gains (Initial Tuning for PSO later)
% Position Control Gains (XY Plane)
Kp_p = diag([100000.0, 100000.0, 0.0]);
Kd_p = diag([1000.0, 1000.0, 0.0]);

% Force Control Gains (Z Axis)
Kp_f = diag([0.0, 0.0, 0.0]);
Ki_f = diag([0.0, 0.0, 3.0]);

% Active Damping for Z-axis impact (Raibert-Craig addition)
Kad_f = 100;

% Derivative filter
N = 50;

% Unified array to be consumed directly by the Simulink blocks
robot_params = [L1; L2; pi_1; pi_2; pi_3; pi_4; pi_5; Im2_eq; Im3_eq; m_load; g];

%disp('Workspace successfully initialized. Ready to run the Simulink model.');