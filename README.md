# Autonomous Vehicle Lateral Control Simulator

A professional-grade simulation environment for benchmarking autonomous vehicle lateral control strategies. This project implements a nonlinear Kinematic Bicycle Model and compares classical PID control with optimization-based Model Predictive Control (MPC).

## Phase 0: System Mathematical Framework

### 1. Kinematic Bicycle Model (Rear-Axle Reference)
For low-speed maneuvers where tire lateral slip is negligible, the vehicle physics are governed by the following nonlinear system of ordinary differential equations (ODEs):

$$\dot{X} = v \cdot \cos(\psi)$$
$$\dot{Y} = v \cdot \sin(\psi)$$
$$\dot{\psi} = \frac{v}{L} \cdot \tan(\delta)$$

Where:
* $X, Y$: Global coordinates of the center of the rear axle (meters).
* $\psi$: Vehicle heading/yaw angle relative to the global X-axis (radians).
* $v$: Constant longitudinal velocity (m/s).
* $\delta$: Front wheel steering angle command (radians).
* $L$: Vehicle wheelbase (meters).

### 2. Numerical Integration Setup
To propagate our continuous-time differential equations forward in a digital computer environment, we utilize a 4th-Order Runge-Kutta (RK4) integration scheme with a fixed time-step ($dt = 0.01$ seconds).

