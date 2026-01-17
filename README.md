# Predictor-Based Adaptive Control for a Hexacopter UAV in Urban Delivery Missions

**Authors:** Andrés T. J., Di Mauro M., Galluzzi G.  
**Institution:** Politecnico di Milano  
**Course:** Adaptive and Autonomous Aerospace Systems  
**Professor:** Davide Invernizzi  
**Academic Year:** 2025/2026  

---

## Overview

This MATLAB/Simulink project develops an adaptive control framework for a hexacopter UAV performing autonomous package delivery in urban environments. The work includes full 6-DoF modeling, baseline control architecture design, Predictor-Based Model Reference Adaptive Control (PBMRAC) augmentation, and trajectory planning via RRT* for obstacle avoidance.

The project integrates simulation, control, and planning inside a unified workflow: a main MATLAB script runs the Simulink model, loads trajectory data, and manages the mission scenario.

---

## Main Topics

- Full 6-DoF hexacopter dynamic modeling
- First-order propeller dynamics
- Trimming at position and velocity setpoints
- Baseline controller design:
- Position control (LQR / eigenvalue assignment)
- Geometric attitude control on SO(3)
- Control allocation via pseudo-inverse for hexacopter actuation
- PBMRAC adaptive augmentation
- Compensation for uncertain payload mass
- Compensation for actuator effectiveness degradation
- Urban delivery scenario setup
- RRT* path planning for 3D obstacle avoidance
- Mission trajectory generation (trajectory data file)
- Testing under disturbances (wind gusts, actuator degradation)
- Performance evaluation: tracking error, settling time, robustness

---

## Requirements

- MATLAB  
- Simulink  

---

## Project Structure
/
├── main.m                       % Main script to run the full simulation
├── DiMauro_Galluzzi_Andres_2024_finalversion.slx         % Simulink model of the UAV and controllers
├── trajectory_data.mat          % Precomputed trajectory for Simulink
├── PlanningPathFunction.m		 % RRT* planner for obstacle avoidance   
├── map_tools/               % Auxiliary mapping and environment tools
├── Adaptive and Autonomous Presentation.ppt             % Full project workflow, analysis, and results

---

## How to Run

Open MATLAB

Run main.m

The script will:

Load all model parameters

Generate or load the RRT* trajectory

Pass trajectory data to the Simulink model

Launch the full simulation

---

## License

Educational purposes – Politecnico di Milano (2025/2026)
