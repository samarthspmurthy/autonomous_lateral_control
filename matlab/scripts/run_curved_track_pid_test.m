% run_curved_track_pid_test.m
% Phase 3: Reference Path Tracking along a Sinusoidal Trajectory
clear; clc; close all;

% 1. Setup Paths and Load Parameters
addpath(fullfile(pwd, '../functions')); 
run('parameters.m');

% --- INITIAL CONDITIONS ---
% Start the car perfectly at the origin this time
X_0 = 0;
Y_0 = 0;       
psi_0 = 0;     
current_state = [X_0; Y_0; psi_0];
v = 8; % Set speed to 8 m/s (~29 km/h) for curved handling

% --- PATH PARAMETERS ---
A = 4.0;         % Amplitude: 4 meters left/right drift
lambda = 50.0;   % Wavelength: Completes a full S-turn every 50 meters

% --- TUNED PID GAINS ---
K_y = 0.25;      % Stronger P gain to handle aggressive curves
K_i = 0.01;      
K_d = 0.15;      
K_psi = 0.6;     
max_steer = 0.6; 

% --- PID MEMORY ---
prev_e_ct = 0;
int_e_ct = 0;

% 2. Pre-allocation
time = 0:dt:t_end;
N = length(time);

X_hist = zeros(1, N);
Y_hist = zeros(1, N);
psi_hist = zeros(1, N);
delta_hist = zeros(1, N);
Y_ref_hist = zeros(1, N); % Track the path geometry too!

X_hist(1) = current_state(1);
Y_hist(1) = current_state(2);
psi_hist(1) = current_state(3);

% 3. The Closed-Loop Tracking Loop
disp('Starting Sinusoidal Track Simulation...');
for i = 1:(N-1)
    
    X_curr = current_state(1);
    Y_curr = current_state(2);
    psi_curr = current_state(3);
    
    % ==========================================
    % 1. DYNAMIC REFERENCE CALCULATION (The Moving Target)
    % ==========================================
    % Calculate what the track is doing at the car's current X position
    Y_target = A * sin((2 * pi / lambda) * X_curr);
    
    % The Paper Derivative: Slope = dY/dX
    slope_target = A * (2 * pi / lambda) * cos((2 * pi / lambda) * X_curr);
    
    % Target Heading: Angle of the curve tangent line
    psi_target = atan(slope_target);
    
    % Save reference path for plotting
    Y_ref_hist(i) = Y_target;
    
    % ==========================================
    % 2. ERROR DYNAMICS COMPUTATION
    % ==========================================
    e_ct = Y_curr - Y_target;
    e_psi = psi_curr - psi_target;
    
    % CRITICAL CONTROL TRICK: Wrap heading error to [-pi, pi] to prevent angle flip bugs
    e_psi = atan2(sin(e_psi), cos(e_psi));
    
    % PID math
    diff_e_ct = (e_ct - prev_e_ct) / dt;
    int_e_ct = int_e_ct + (e_ct * dt);
    prev_e_ct = e_ct;
    
    % Control Law
    delta_cmd = -(K_y * e_ct + K_i * int_e_ct + K_d * diff_e_ct + K_psi * e_psi);
    
    % Saturation Clamp
    if delta_cmd > max_steer, delta_cmd = max_steer; end
    if delta_cmd < -max_steer, delta_cmd = -max_steer; end
    
    delta_hist(i) = delta_cmd;
    
    % ==========================================
    % 3. THE PLANT (RK4 Physics Update)
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
Y_ref_hist(N) = A * sin((2 * pi / lambda) * X_hist(N));
delta_hist(N) = delta_hist(N-1);
disp('Simulation Complete!');

% 4. Trajectory Tracking Dashboard
figure('Name', 'Phase 3: Sinusoidal Path Tracking', 'Position', [100, 100, 1100, 450]);

% Plot 1: Path Follow Map
subplot(2, 1, 1);
plot(X_hist, Y_ref_hist, 'r--', 'LineWidth', 2); hold on;
plot(X_hist, Y_hist, 'b-', 'LineWidth', 2);
grid on;
xlabel('Global X Position (meters)');
ylabel('Global Y Position (meters)');
title('Vehicle Tracking a Sinusoidal Lane Change');
legend('Target Road Path', 'Actual Vehicle Trajectory', 'Location', 'best');
axis equal;

% Plot 2: Actuator Steering Command Over Time
subplot(2, 1, 2);
plot(time, delta_hist * (180/pi), 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (seconds)');
ylabel('Steering Angle (degrees)');
title('Dynamic Steering Effort Through S-Turns');