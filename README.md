# Rotational Dynamics of a Rigid Body

This MATLAB project simulates the rotational dynamics of a rigid body in space, using both **Euler's equations of motion** and **quaternion-based attitude kinematics**. The simulation captures the evolution of angular velocity and orientation under torque-free or torque-driven motion.

---

## Overview

This repository solves the coupled nonlinear differential equations for:

- **Rotational motion** using Euler’s equations:
  \[
  \mathbf{I} \cdot \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I} \cdot \boldsymbol{\omega}) = \boldsymbol{\tau}
  \]

- **Attitude kinematics** using **quaternions**, which are more numerically stable than Euler angles or DCMs alone.

The simulation uses `ode45` to numerically integrate these equations over time and provides visualizations of angular velocity and orientation.

---

## Repository Contents

```text
Rotational_dynamics/
├── main.m              % Main driver script
├── rot_rigid_ode.m     % Euler’s equations for rotational dynamics
├── attitude_ode.m      % Quaternion kinematic equations
├── euler_quat.m        % Converts Euler angles to quaternions
├── dcm_matrix.m        % Converts quaternion to DCM (rotation matrix)
