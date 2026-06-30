% run_open_loop_test.m
% Phase 1: Open-Loop Simulation using RK4 Integration
clear; clc; close all;

% 1. Setup Paths and Load Parameters
% (Tells MATLAB where to find our physics factory)
addpath(fullfile(pwd, '../functions')); 
run('parameters.m');

% 2. Initialization & Pre-allocation
% Create a time array from 0 to t_end jumping by dt
time = 0:dt:t_end;
N = length(time);

% Pre-allocate empty arrays to record the car's history for plotting
X_hist = zeros(1, N);
Y_hist = zeros(1, N);
psi_hist = zeros(1, N);

% Set our starting point [0; 0; 0]
current_state = [X_0; Y_0; psi_0];

% Record the starting point at t=0
X_hist(1) = current_state(1);
Y_hist(1) = current_state(2);
psi_hist(1) = current_state(3);

% 3. The RK4 Integration Loop
disp('Starting RK4 Simulation Loop...');
for i = 1:(N-1)
    
    % Scout 1: The Start Line
    k1 = bicycle_model_rear(current_state, v, delta, L);
    
    % Scout 2: The Halfway Test
    state_k2 = current_state + k1 * (dt / 2);
    k2 = bicycle_model_rear(state_k2, v, delta, L);
    
    % Scout 3: The Second Halfway Test
    state_k3 = current_state + k2 * (dt / 2);
    k3 = bicycle_model_rear(state_k3, v, delta, L);
    
    % Scout 4: The Finish Line Test
    state_k4 = current_state + k3 * dt;
    k4 = bicycle_model_rear(state_k4, v, delta, L);
    
    % THE UPDATE: Calculate weighted average and OVERWRITE the current_state
    current_state = current_state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
    
    % Save the new state to our history log so we can graph it later
    X_hist(i+1) = current_state(1);
    Y_hist(i+1) = current_state(2);
    psi_hist(i+1) = current_state(3);
    
end
disp('Simulation Complete!');

% 4. Quick Visualization (The Satellite Map)
figure;
plot(X_hist, Y_hist, 'b-', 'LineWidth', 2);
grid on;
xlabel('Global X Position (meters)');
ylabel('Global Y Position (meters)');
title('Open-Loop Vehicle Path (Constant Steering)');
axis equal; % CRITICAL: Forces X and Y grids to be squares so curves look physically real