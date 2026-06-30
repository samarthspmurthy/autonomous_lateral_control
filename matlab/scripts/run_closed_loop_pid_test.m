% run_closed_loop_pid_test.m
% Phase 2: Closed-Loop Control (Tracking a straight line at Y = 0)
clear; clc; close all;

% 1. Setup Paths and Load Parameters
addpath(fullfile(pwd, '../functions')); 
run('parameters.m');

% --- OVERRIDE INITIAL CONDITIONS FOR OUR TEST SCENARIO ---
% Let's start the car exactly where we put it in our paper dry run!
X_0 = 0;
Y_0 = 2.0;       % 2 meters to the left
psi_0 = 0.1;     % Pointing slightly left
current_state = [X_0; Y_0; psi_0];

% --- CONTROLLER GAINS ---
K_y = 0.15;      % Cross-track error gain
K_psi = 0.5;     % Heading error gain
max_steer = 0.6; % Limit steering to ~34 degrees for physical realism

% 2. Pre-allocation
time = 0:dt:t_end;
N = length(time);

X_hist = zeros(1, N);
Y_hist = zeros(1, N);
psi_hist = zeros(1, N);
delta_hist = zeros(1, N); % We need to track the steering effort!

X_hist(1) = current_state(1);
Y_hist(1) = current_state(2);
psi_hist(1) = current_state(3);
delta_hist(1) = 0; 

% Target path definition (Straight line on X-axis)
Y_target = 0;
psi_target = 0;

% 3. The Closed-Loop RK4 Integration
disp('Starting Closed-Loop Control Simulation...');
for i = 1:(N-1)
    
    % ==========================================
    % 1. THE CONTROLLER (The Brain)
    % ==========================================
    % Read current state
    current_Y = current_state(2);
    current_psi = current_state(3);
    
    % Calculate Errors (The Geometry)
    e_ct = current_Y - Y_target;
    e_psi = current_psi - psi_target;
    
    % Compute Steering Command (The Negative Feedback Law)
    delta_cmd = -(K_y * e_ct + K_psi * e_psi);
    
    % Hardware Clamp: Prevent physically impossible steering angles
    if delta_cmd > max_steer
        delta_cmd = max_steer;
    elseif delta_cmd < -max_steer
        delta_cmd = -max_steer;
    end
    
    % Record steering command for plotting later
    delta_hist(i) = delta_cmd;
    
    % ==========================================
    % 2. THE PLANT (The Physics)
    % ==========================================
    % Notice how delta_cmd is passed to the RK4 scouts!
    
    k1 = bicycle_model_rear(current_state, v, delta_cmd, L);
    
    state_k2 = current_state + k1 * (dt / 2);
    k2 = bicycle_model_rear(state_k2, v, delta_cmd, L);
    
    state_k3 = current_state + k2 * (dt / 2);
    k3 = bicycle_model_rear(state_k3, v, delta_cmd, L);
    
    state_k4 = current_state + k3 * dt;
    k4 = bicycle_model_rear(state_k4, v, delta_cmd, L);
    
    % THE UPDATE: Move the car forward in time
    current_state = current_state + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
    
    % Log state
    X_hist(i+1) = current_state(1);
    Y_hist(i+1) = current_state(2);
    psi_hist(i+1) = current_state(3);
    
end
% Fill last delta for clean plotting
delta_hist(N) = delta_hist(N-1); 
disp('Simulation Complete!');

% 4. Visualization Dashboard
figure('Name', 'Phase 2: Closed-Loop Tracking', 'Position', [100, 100, 1000, 400]);

% Plot 1: The Global Map (X vs Y)
subplot(1, 2, 1);
plot(X_hist, Y_hist, 'b-', 'LineWidth', 2); hold on;
plot([0, max(X_hist)], [0, 0], 'r--', 'LineWidth', 2); % The Target Line
grid on;
xlabel('Global X Position (meters)');
ylabel('Global Y Position (meters)');
title('Vehicle Path vs Target Line');
legend('Actual Vehicle Path', 'Target Path (Y=0)', 'Location', 'best');
axis equal;

% Plot 2: Steering Effort Over Time
subplot(1, 2, 2);
plot(time, delta_hist * (180/pi), 'k-', 'LineWidth', 2); % Convert rad to deg for easy reading
grid on;
xlabel('Time (seconds)');
ylabel('Steering Angle (degrees)');
title('Controller Steering Effort');