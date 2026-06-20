% MAIN_PHASE5: Multi-Obstacle Autonomous Avoidance

clc; clear; close all;

%% 1. Initialization
dt = 0.5;           
t_end = 150; % Increased time for a much longer 1000m path
time = 0:dt:t_end;  
V = 13;             

initial_state = [0; 100; -20; 0]; 
goal_pos = [1000; 1100; -100];      % Pushed the goal much further away

state_history = zeros(4, length(time));
state_history(:, 1) = initial_state;
current_state = initial_state;

% MULTIPLE OBSTACLES: Spaced out over 1000 meters
obs_positions = [150,  300,  450,  600,  750,  850;  % North
                 200,  450,  500,  750,  800, 1000;  % East
                 -30,  -40,  -50,  -60,  -70,  -80]; % Down (Height)
             
num_obs = size(obs_positions, 2);
obs_radius = 20; 

% EKF Setup
R_var = 0.0012;
noise_std = [sqrt(R_var); sqrt(R_var)]; 
Q = diag([0.00001, 0.0001, 0.0001]);
R = diag([R_var, R_var]);
tau_0 = 0.06;

% We now need a matrix of states and a cell array of covariances
x_est = zeros(3, num_obs);
P_est = cell(1, num_obs);

for j = 1:num_obs
    z_initial = virtual_camera(current_state, obs_positions(:, j), noise_std);
    x_est(:, j) = [tau_0; z_initial(1); z_initial(2)]; 
    P_est{j} = diag([0.03^2, R_var, R_var]);
end

% Planner Setup 
m_horizon = 3; 
phi_max = deg2rad(30);
theta_max = deg2rad(15);

lb = repmat([-phi_max, -theta_max], 1, m_horizon);
ub = repmat([phi_max,  theta_max], 1, m_horizon);
options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');

%% 2. Main Simulation Loop
disp('Simulating multi-obstacle autonomous flight...');

for i = 1:(length(time) - 1)
    
    % --- 1. Path Planning (Multi-Obstacle) ---
    U0 = zeros(1, 2 * m_horizon); 
    
    U_opt = fmincon(@(U) planner_cost_multi(U, current_state, x_est, V, dt, obs_radius, goal_pos, obs_positions), ...
                    U0, [], [], [], [], lb, ub, [], options);
    
    u_apply = [U_opt(1); U_opt(2)];
    
    % --- 2. MAV Moves ---
    state_dot = mav_kinematics(0, current_state, u_apply, V);
    current_state = current_state + state_dot * dt;
    state_history(:, i+1) = current_state;
    
    if norm(current_state(1:3) - goal_pos) < 30
        disp('Goal Reached!');
        state_history = state_history(:, 1:i+1); 
        break;
    end
    
    % --- 3. Camera & EKF (For EACH Obstacle) ---
    for j = 1:num_obs
        z = virtual_camera(current_state, obs_positions(:, j), noise_std);
        [x_j, P_j] = ekf_step(x_est(:, j), P_est{j}, z, u_apply, V, dt, Q, R);
        
        x_est(:, j) = x_j;
        P_est{j} = P_j;
    end
end

disp('Simulation Complete.');

%% 3. Visualization
Pn_hist = state_history(1, :);
Pe_hist = state_history(2, :);
Pd_hist = state_history(3, :);

% =========================================================================
% FIGURE 1: Global Trajectory (Figure 5 reproduction)
% =========================================================================
figure('Name', 'Paper Reproduction: Figure 5 (Global Trajectory)', 'Color', 'w');
plot3(Pe_hist, Pn_hist, -Pd_hist, 'b-', 'LineWidth', 2); hold on; grid on;
plot3(Pe_hist(1), Pn_hist(1), -Pd_hist(1), 'go', 'MarkerFaceColor', 'g');
plot3(goal_pos(2), goal_pos(1), -goal_pos(3), 'k*', 'MarkerSize', 10);

% Plot Multiple Obstacles (Reduced resolution to prevent lag)
[X, Y, Z] = sphere(10); 
colors = ['r', 'm', 'c', 'y', 'g', 'b'];

for j = 1:num_obs
    surf(obs_positions(2,j) + X*obs_radius, obs_positions(1,j) + Y*obs_radius, -(obs_positions(3,j) + Z*obs_radius), ...
        'FaceColor', colors(mod(j-1,length(colors))+1), 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end

xlabel('East (m)'); ylabel('North (m)'); zlabel('Height (m)');
title('Global 3D Trajectory (Fig 5)');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k');
view(3); axis equal;

% =========================================================================
% FIGURE 2: Local-Level Frame Map in Spherical Coordinates (Figure 2)
% =========================================================================
figure('Name', 'Paper Reproduction: Figure 2 (Local Map)', 'Color', 'w');
hold on; 

% --- Draw Circular / Polar Grid (Radar Map look) ---
% The paper's map has a circular/spherical aesthetic
theta_grid = linspace(0, 2*pi, 100);
for r_grid = 50:50:300
    % Concentric circles in the XY plane (Height = 0)
    plot3(r_grid*sin(theta_grid), r_grid*cos(theta_grid), zeros(size(theta_grid)), 'Color', [0.8 0.8 0.8], 'LineStyle', '--');
end
% Radial spokes
for angle = 0:pi/4:(2*pi - pi/4)
    plot3([0, 300*sin(angle)], [0, 300*cos(angle)], [0, 0], 'Color', [0.8 0.8 0.8], 'LineStyle', '--');
end

% The origin of the local map is the MAV's position
plot3(0, 0, 0, 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Use the state and EKF estimates from the VERY FIRST step so obstacles 
% are in front of the MAV, just like in the paper.
snap_state = state_history(:, 1);
snap_x_est = zeros(3, num_obs);
snap_P_est = cell(1, num_obs);
for j = 1:num_obs
    z_init = virtual_camera(snap_state, obs_positions(:, j), noise_std);
    snap_x_est(:, j) = [0.06; z_init(1); z_init(2)];
    snap_P_est{j} = diag([0.03^2, R_var, R_var]);
end

for j = 1:num_obs
    tau_est = snap_x_est(1, j);
    eta_est = snap_x_est(2, j);
    xi_est  = snap_x_est(3, j);
    sigma_tau = sqrt(snap_P_est{j}(1,1));
    
    % True position of the obstacle relative to the MAV's snapshot position
    R_yaw = [cos(snap_state(4)), sin(snap_state(4)), 0;
            -sin(snap_state(4)), cos(snap_state(4)), 0; 
             0, 0, 1];
    rel_local = R_yaw * (obs_positions(:, j) - snap_state(1:3));
    
    % Plot the true obstacle location as a red dot
    plot3(rel_local(2), rel_local(1), -rel_local(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Generate 95% uncertainty bounds (+/- 2 sigma)
    tau_samples = linspace(tau_est - 2*sigma_tau, tau_est + 2*sigma_tau, 100);
    
    % Vectorized plotting to eliminate 3D rotation lag
    valid_tau = tau_samples(tau_samples > 0.005);
    r_k = V ./ valid_tau; 
    x_k = r_k .* cos(xi_est) .* cos(eta_est); 
    y_k = r_k .* cos(xi_est) .* sin(eta_est); 
    z_k = r_k .* sin(xi_est);                
    
    % Plot all blue dots for this obstacle at once
    plot3(y_k, x_k, -z_k, 'b.', 'MarkerSize', 4);
end

xlabel('Right wing direction (m)'); ylabel('Heading direction (m)'); zlabel('Height (m)');
title('Local-Level Map in Spherical Coordinates (Fig 2)');
legend('Radar Rings', 'Radial Spokes', 'MAV', 'True Obstacles', '95% Uncertainty (Blue Dots)');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k');
view(-30, 30); axis equal; grid off;
set(gca, 'XDir', 'reverse'); % To match standard local-level visual representation where front is up
