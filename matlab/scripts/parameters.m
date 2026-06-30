%% AV Lateral Control — Open Loop Simulation Parameters
% Clear the workspace and command window
clear; clc;

%% 1. Vehicle Geometric Parameters (Section 1.3.1)
L = 2.5;         % Wheelbase of the vehicle (meters)

%% 2. Simulation Constants & Inputs
v = 5;           % Constant longitudinal velocity (m/s) (~18 km/h)
delta = 0.1;     % Constant front steering angle (radians) (~5.7 degrees)
dt = 0.01;       % Integration time step (seconds) - crucial for RK4
t_end = 10;      % Total simulation time (seconds)

%% 3. Ground-Truth Initial State Vector [X_0; Y_0; psi_0] (Section 1.3.2)
X_0 = 0;         % Initial Global X position (meters)
Y_0 = 0;         % Initial Global Y position (meters)
psi_0 = 0;       % Initial Heading/Yaw angle (radians)

disp('Parameters loaded successfully!');