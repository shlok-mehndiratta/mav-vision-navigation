% MAIN_PHASE2: Simulates MAV kinematics, an Obstacle, and Virtual Camera

%% 1. Initialization
clc; clear; close all;

dt = 0.1;           
t_end = 20;         
time = 0:dt:t_end;  

V = 13;             

initial_state = [120; 120; -60; 0]; 
state_history = zeros(4, length(time));
state_history(:, 1) = initial_state;
current_state = initial_state;

% --- NEW: Obstacle Definition ---
% Let's place an obstacle directly in the MAV's path
obs_pos = [300; 150; -60]; % [North, East, Down]
obs_radius = 20; % 20 meters (from the paper)

% --- NEW: Camera Settings ---
% Standard deviation for camera noise (2 degrees, converted to radians)
noise_std = [deg2rad(2); deg2rad(2)]; 

% To store our noisy camera measurements
camera_measurements = zeros(2, length(time)-1);
true_azimuth_history = zeros(1, length(time)-1); % for comparison later

%% 2. Main Simulation Loop
for i = 1:(length(time) - 1)
    
    % MAV Control Input
    phi = deg2rad(15); 
    theta = deg2rad(5); 
    u = [phi; theta];
    
    % Calculate derivatives and update state
    state_dot = mav_kinematics(time(i), current_state, u, V);
    current_state = current_state + state_dot * dt;
    state_history(:, i+1) = current_state;
    
    % --- Take a picture with the Virtual Camera ---
    % Get the noisy [azimuth; elevation]
    z = virtual_camera(current_state, obs_pos, noise_std);
    camera_measurements(:, i) = z;
    
    % Let's quickly save the true azimuth to see the noise effect later
    R_yaw = [cos(current_state(4)), sin(current_state(4)), 0;
            -sin(current_state(4)), cos(current_state(4)), 0; 0, 0, 1];
    rel = R_yaw * (obs_pos - current_state(1:3));
    true_azimuth_history(i) = atan2(rel(2), rel(1));
end

%% 3. Visualization
Pn_hist = state_history(1, :);
Pe_hist = state_history(2, :);
Pd_hist = state_history(3, :);

% Figure 1: 3D Trajectory
figure('Name', 'Phase 2: MAV and Obstacle', 'Color', 'w');
plot3(Pe_hist, Pn_hist, -Pd_hist, 'b-', 'LineWidth', 2); hold on; grid on;
plot3(Pe_hist(1), Pn_hist(1), -Pd_hist(1), 'go', 'MarkerFaceColor', 'g');

% Plot the Obstacle as a red sphere
[X, Y, Z] = sphere(20); 
surf(obs_pos(2) + X*obs_radius, obs_pos(1) + Y*obs_radius, -(obs_pos(3) + Z*obs_radius), ...
    'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

xlabel('East (m)'); ylabel('North (m)'); zlabel('Height (m)');
title('MAV Trajectory and Obstacle');
legend('Trajectory', 'Start', 'Obstacle');
view(3); axis equal;

% Figure 2: Camera Data
figure('Name', 'Phase 2: Camera Measurements', 'Color', 'w');
plot(time(1:end-1), rad2deg(camera_measurements(1, :)), 'r.', 'MarkerSize', 8); hold on;
plot(time(1:end-1), rad2deg(true_azimuth_history), 'k-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Azimuth Angle (degrees)');
title('Virtual Camera Output vs True Azimuth');
legend('Noisy Camera Measurements', 'True Azimuth');
