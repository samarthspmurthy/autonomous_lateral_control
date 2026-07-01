% run_pure_pursuit_track_test.m
% Phase 3: Geometric Path Tracking using Pure Pursuit
clear; clc; close all;

% 1. Setup Paths and Load Parameters FIRST
addpath(fullfile(pwd, '../functions')); 
run('parameters.m');  % <--- This securely defines dt and t_end for N!

% --- INITIAL CONDITIONS ---
X_0 = 0;
Y_0 = 0;       
psi_0 = 0;     
current_state = [X_0; Y_0; psi_0];
v = 8; % Speed in m/s

% --- PATH PARAMETERS ---
A = 4.0;         
lambda = 50.0;   

% --- PURE PURSUIT TUNING PARAMETERS ---
lf = 6.0;        % Look-Ahead Distance: 6 meters out
max_steer = 0.6; % Limits steering to ~34 degrees

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

% Pre-generate a high-resolution reference track map to look ahead into
X_ref_map = 0:0.1:(v * t_end + 20);
Y_ref_map = A * sin((2 * pi / lambda) * X_ref_map);

disp('Starting Pure Pursuit Geometric Tracking Simulation...');
for i = 1:(N-1)
    
    X_curr = current_state(1);
    Y_curr = current_state(2);
    psi_curr = current_state(3);
    
    % ==========================================
    % 1. GEOMETRIC LOOK-AHEAD SEARCH
    % ==========================================
    % Calculate distances from current rear axle to all points on the track map
    distances = sqrt((X_ref_map - X_curr).^2 + (Y_ref_map - Y_curr).^2);
    
    % Find the point on the track closest to our look-ahead distance (lf)
    % but ensuring it is ahead of the vehicle position
    valid_indices = find(X_ref_map >= X_curr);
    if isempty(valid_indices)
        valid_indices = length(X_ref_map);
    end
    
    [~, min_idx_local] = min(abs(distances(valid_indices) - lf));
    goal_idx = valid_indices(min_idx_local);
    
    X_goal = X_ref_map(goal_idx);
    Y_goal = Y_ref_map(goal_idx);
    
    % ==========================================
    % 2. COORDINATE TRANSFORMATION TO VEHICLE FRAME
    % ==========================================
    % Translate reference coordinates relative to vehicle center
    dx = X_goal - X_curr;
    dy = Y_goal - Y_curr;
    
    % Rotate into vehicle's local frame (Heading alignment)
    % local_x is forward, local_y is lateral shift
    local_y = -dx * sin(psi_curr) + dy * cos(psi_curr);
    
    % Calculate alpha (angle to goal point in local frame)
    alpha = atan2(local_y, lf);
    
    % ==========================================
    % 3. GEOMETRIC CONTROL LAW
    % ==========================================
    delta_cmd = atan2(2 * L * sin(alpha), lf);
    
    % Saturation Clamp (Clean syntax block)
    if delta_cmd > max_steer
        delta_cmd = max_steer;
    elseif delta_cmd < -max_steer
        delta_cmd = -max_steer;
    end
    
    delta_hist(i) = delta_cmd;
    
    % ==========================================
    % 4. THE PLANT (RK4 Physics Update)
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

% 3. Pure Pursuit Performance Dashboard
figure('Name', 'Phase 3: Pure Pursuit Geometric Tracking', 'Position', [150, 150, 1100, 450]);

subplot(2, 1, 1);
plot(X_ref_map(1:find(X_ref_map > X_hist(end), 1)), Y_ref_map(1:find(X_ref_map > X_hist(end), 1)), 'r--', 'LineWidth', 2); hold on;
plot(X_hist, Y_hist, 'b-', 'LineWidth', 2);
grid on;
xlabel('Global X Position (meters)');
ylabel('Global Y Position (meters)');
title('Vehicle Tracking via Pure Pursuit Look-Ahead Geometry');
legend('Target Road Path', 'Actual Pure Pursuit Trajectory', 'Location', 'best');
axis equal;

subplot(2, 1, 2);
plot(time, delta_hist * (180/pi), 'm-', 'LineWidth', 1.5);
grid on;
xlabel('Time (seconds)');
ylabel('Steering Angle (degrees)');
title('Geometric Steering Command History');