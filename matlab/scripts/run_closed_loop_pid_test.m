% run_closed_loop_pid_test.m
% Phase 2.1: Full PID Closed-Loop Control (High Speed Tracking)
clear; clc; close all;

% 1. Setup Paths and Load Parameters
addpath(fullfile(pwd, '../functions')); 
run('parameters.m');

% --- OVERRIDE INITIAL CONDITIONS FOR HIGH-SPEED TEST ---
X_0 = 0;
Y_0 = 2.0;       % 2 meters to the left
psi_0 = 0.1;     % Pointing slightly left
current_state = [X_0; Y_0; psi_0];
v = 15;          % OVERRIDE: Force high speed (54 km/h) to test the D-term!

% --- PID CONTROLLER GAINS ---
K_y = 0.15;      % Proportional (P) - The Steer
K_i = 0.05;      % Integral (I) - The Bucket
K_d = 0.25;      % Derivative (D) - The Brake
K_psi = 0.5;     % Heading error gain
max_steer = 0.6; % Limit steering to ~34 degrees

% --- INITIALIZE PID MEMORY VARIABLES ---
prev_e_ct = Y_0 - 0; % Memory of the previous error
int_e_ct = 0;        % Empty bucket at t=0

% 2. Pre-allocation
time = 0:dt:t_end;
N = length(time);

X_hist = zeros(1, N);
Y_hist = zeros(1, N);
psi_hist = zeros(1, N);
delta_hist = zeros(1, N);

X_hist(1) = current_state(1);
Y_hist(1) = current_state(2);
psi_hist(1) = current_state(3);
delta_hist(1) = 0; 

Y_target = 0;
psi_target = 0;

% 3. The Closed-Loop RK4 Integration
disp('Starting Full PID Simulation Loop...');
for i = 1:(N-1)
    
    % ==========================================
    % 1. THE CONTROLLER (The Full PID Brain)
    % ==========================================
    current_Y = current_state(2);
    current_psi = current_state(3);
    
    % Calculate Current Errors
    e_ct = current_Y - Y_target;
    e_psi = current_psi - psi_target;
    
    % Calculate Derivative (Rate of change: slope over dt)
    diff_e_ct = (e_ct - prev_e_ct) / dt;
    
    % Calculate Integral (Accumulated error: rectangle area)
    int_e_ct = int_e_ct + (e_ct * dt);
    
    % The Full PID Control Law (Negative Feedback)
    delta_cmd = -(K_y * e_ct + K_i * int_e_ct + K_d * diff_e_ct + K_psi * e_psi);
    
    % Hardware Clamp
    if delta_cmd > max_steer
        delta_cmd = max_steer;
    elseif delta_cmd < -max_steer
        delta_cmd = -max_steer;
    end
    
    % CRITICAL: Save current error to become the "previous" error for the NEXT loop iteration
    prev_e_ct = e_ct;
    
    % Record steering command
    delta_hist(i) = delta_cmd;
    
    % ==========================================
    % 2. THE PLANT (The Physics)
    % ==========================================
    k1 = bicycle_model_rear(current_state, v, delta_cmd, L);
    
    state_k2 = current_state + k1 * (dt / 2);
    k2 = bicycle_model_rear(state_k2, v, delta_cmd, L);
    
    state_k3 = current_state + k2 * (dt / 2);
    k3 = bicycle_model_rear(state_k3, v, delta_cmd, L);
    
    state_k4 = current_state + k3 * dt;
    k4 = bicycle_model_rear(state_k4, v, delta_cmd, L);
    
    current_state = current_state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
    
    X_hist(i+1) = current_state(1);
    Y_hist(i+1) = current_state(2);
    psi_hist(i+1) = current_state(3);
    
end
delta_hist(N) = delta_hist(N-1); 
disp('Simulation Complete!');

% 4. Visualization Dashboard
figure('Name', 'Phase 2.1: Full PID Tracking', 'Position', [100, 100, 1000, 400]);

subplot(1, 2, 1);
plot(X_hist, Y_hist, 'b-', 'LineWidth', 2); hold on;
plot([0, max(X_hist)], [0, 0], 'r--', 'LineWidth', 2); 
grid on;
xlabel('Global X Position (meters)');
ylabel('Global Y Position (meters)');
title('Vehicle Path at 15 m/s (Full PID)');
legend('Actual Path', 'Target Path (Y=0)', 'Location', 'best');
axis equal;

subplot(1, 2, 2);
plot(time, delta_hist * (180/pi), 'k-', 'LineWidth', 2); 
grid on;
xlabel('Time (seconds)');
ylabel('Steering Angle (degrees)');
title('Controller Steering Effort');