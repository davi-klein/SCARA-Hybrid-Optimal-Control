# SCARA Hybrid Position/Force Control Simulation

## Overview
This repository contains the development, mathematical modeling, and simulation of a robust control architecture for a 3-Degree-of-Freedom (DOF) SCARA robotic manipulator (RRP architecture). The system is designed to operate in tasks involving physical interaction with the environment, such as robotic polishing. 

The main challenge addressed is the tracking of a trochoidal trajectory in free space (XY Cartesian plane) simultaneous to the descent, impact, and stabilization of a continuous force against a rigid surface along the Z-axis. The project successfully implements both the physical baseline and an advanced Particle Swarm Optimization (PSO) layer to tune the controller.

## System Architecture
The logical and physical foundation of the system is fully implemented in MATLAB/Simulink.

* **Physical Plant (Digital Twin):** Multibody modeling including inertia matrices, Coriolis forces, gravitational compensation, and friction (viscous and Coulomb), based on the experimental parameter identification establsihed by Boschetti and Sinico (2025).
* **Contact Dynamics:** Implementation of a Kelvin-Voigt viscoelastic unilateral contact model to simulate the interaction with the surface. The `ode15s` (stiff) numerical solver is mandatory to ensure stability given the extreme stiffness of the differential equations at the moment of collision.
* **Hybrid Control:** Decoupling of the operational space using orthogonal selection matrices though the Raibert-Craig architecture. A PD controller tracks the planar kinematics, and a PI controller coupled with an active damping term ($k_{ad}$) regulates the Z-axis force.
* **PSO Optimization:** A metaheuristic search loop operating asynchronously over the simulation. It utilizes a multiobjective cost function with exterior quadratic penalties to ensure parametric evolution strictly respects the hardware's inherent torque, velocity, and positional limits.

## Repository Structure & Execution
The architecture maintains a strict separation between code-based parameterization, loop integration, and data visualization:

```text
├── data/
│   └──                       # Contains optimized gains and optimization result logs
├── src/
│   ├── init.m                # Allocates physical parameters and baseline empirical gains
│   ├── cost_function_scara.m # Evaluates the hybrid control performance and constraints
│   └── SCARAHybCtrl.slx      # Main Simulink model (Kinematics, Dynamics, Environment, Control)
├── main_pso.m                # Executes the swarm optimization asynchronously from the root
└── README.md                 # Technical documentation
```

**Execution Procedure:**
1. Clone the repository to your local environment.
2. In MATLAB, change the working directory to the project root.
3. Execute `main_pso.m` to run the optimization, or open `src/SCARAHybCtrl.slx` to simulate with baseline parameters (for pure simulaitons ensure the solver is set to `ode15s`, and the src/init.m script has been ran).

## Optimization Results
The implementation of the PSO algorithm yielded significant performance improvements over the empirical baseline. The optimized gains drastically reduced the Root-Mean-Square (RMS) tracking error, dropping the planar error e_x from 0.1296 mm to 0.0378 mm. 

Furthermore, the constraint handling methodology proved successful. The optimized active damping gain generated a precisely timed opposing force (peaking safely at 52.92 N, well below the 150 N actuator limit) to dissipate kinetic energy upon rigid impact, effectively suppressing the jackhammering effect while keeping rotational joint torques within their stringent saturation bounds.

---
**Author:**  
**Davi Klein**  
M.S in Mechanical Engineering (UFSC) | M.S. Student in Computer Science | AI & Robotics Researcher  
Federal University of Santa Maria (UFSM)