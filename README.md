# SCARA Hybrid Position/Force Control Simulation

## Overview
This repository contains the development, mathematical modeling, and simulation of a robust control architecture for a 3-Degree-of-Freedom (DOF) SCARA robotic manipulator (RRP architecture).

The system is designed to operate in tasks involving physical interaction with the environment. The main challenge addressed is the tracking of a circular trajectory in free space (XY Cartesian plane) simultaneous to the descent, impact, and stabilization of a continuous force against a rigid surface along the Z-axis.

The project is divided into two main phases: the construction of the physical and control baseline (Completed) and the development of the parameter optimization layer (In Progress).

## System Architecture and Current Status
The logical and physical foundation of the system is fully implemented in Simulink, operating with empirically defined control gains.

* **Physical Plant (Digital Twin):** Multibody modeling including inertia matrices, Coriolis forces, gravitational compensation, and friction (viscous and Coulomb).
* **Contact Dynamics:** Implementation of a viscoelastic unilateral contact model to simulate the interaction with the surface. The `ode15s` (stiff) numerical solver is mandatory to ensure stability given the extreme stiffness of the differential equations at the moment of collision.
* **Hybrid Control:** Decoupling of the operational space using orthogonal selection matrices. A PD controller is applied for tracking the planar kinematics, and a PI controller coupled with a feedforward term is used for force regulation on the Z-axis.
* **Trajectory Generator:** Generation of continuous references (C2) to smooth state variables and mitigate transients.

## Repository Structure
The architecture maintains a strict separation between code-based parameterization and loop integration:

```text
.
├── init.m              # Initialization script responsible for allocating physical parameters and control gains in the workspace.
├── SCARAHybCtrl.slx    # Main Simulink model containing Kinematics, Dynamics, Environment, and the Hybrid Controller.
└── README.md           # Technical documentation of the project.
```

## Execution Instructions

**Prerequisites:**
* MATLAB
* Simulink

**Execution Procedure:**
1. Clone the repository to your local environment.
2. In MATLAB, change the working directory to the project root.
3. Run the `init.m` script in the console to load the tensors and state variables.
4. Open the `SCARAHybCtrl.slx` model.
5. Start the simulation. Ensure the solver configuration is set to `ode15s` in the model properties to prevent integration collapse during rigid contact.

## Work in Progress
The current development effort focuses on replacing empirical gains with a tuning architecture based on iterative computational methods. The goal is to determine the optimal stiffness matrix of the controller to attenuate high-frequency oscillations during phase transition (free motion to contact).

**Implementation Roadmap:**

* [ ] **Performance Evaluation Module:** Construction of a cost function based on a scalarized multiobjective criterion, measuring geometric positional error and force error in transient and steady states.
* [ ] **Physical Constraints Handling:** Application of smooth quadratic penalty functions to ensure parametric evolution does not exceed the hardware's inherent torque, velocity, and acceleration limits.
* [ ] **Optimizer Script:** Integration of the metaheuristic search loop operating in parallel and asynchronously over the control loop simulations.  

## Author

**Davi Klein**  
M.S in Mechanichal Engineering (UFSC) | M.S. Student in Computer Science | AI & Robotics Researcher  
Federal University of Santa Maria (UFSM)
