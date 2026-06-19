% MAIN_PHASE4: Full Autonomous Obstacle Avoidance

clc; clear; close all;

%% 1. Initialization
dt = 0.5;           % Increased dt slightly to speed up optimization calculations
t_end = 90;         
time = 0:dt:t_end;  
V = 13;             

initial_state = [0; 100; -20; 0]; % Start position from paper
goal_pos = [600; 700; -100];      % End position from paper

state_history = zeros(4, length(time));
state_history(:, 1) = initial_state;
current_state = initial_state;

% Obstacle right in the direct path to the goal
obs_pos = [300; 400; -60]; 
obs_radius = 20; 

% EKF Setup
R_var = 0.0012;
noise_std = [sqrt(R_var); sqrt(R_var)]; 
Q = diag([0.00001, 0.0001, 0.0001]);
R = diag([R_var, R_var]);
tau_0 = 0.06;
P_est = diag([0.03^2, R_var, R_var]);
z_initial = virtual_camera(current_state, obs_pos, noise_std);
x_est = [tau_0; z_initial(1); z_initial(2)]; 

% Planner Setup (m-step look-ahead)
m_horizon = 3; 
phi_max = deg2rad(30);
theta_max = deg2rad(15);

% Control bounds: [phi1, theta1, phi2, theta2, phi3, theta3]
lb = repmat([-phi_max, -theta_max], 1, m_horizon);
ub = repmat([phi_max,  theta_max], 1, m_horizon);

% fmincon optimization settings (suppress printing to make it run faster)
options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');

%% 2. Main Simulation Loop
disp('Simulating autonomous flight... This may take a moment due to optimization.');

for i = 1:(length(time) - 1)
    
    % --- 1. Path Planning ---
    % Initial guess for the optimizer (straight flight)
    U0 = zeros(1, 2 * m_horizon); 
    
    % fmincon finds the optimal controls U_opt that minimize our planner_cost function
    U_opt = fmincon(@(U) planner_cost(U, current_state, x_est, V, dt, obs_radius, goal_pos), ...
                    U0, [], [], [], [], lb, ub, [], options);
    
    % We only apply the very first control step (Receding Horizon Control)
    u_apply = [U_opt(1); U_opt(2)];
    
    % --- 2. MAV Moves ---
    state_dot = mav_kinematics(0, current_state, u_apply, V);
    current_state = current_state + state_dot * dt;
    state_history(:, i+1) = current_state;
    
    % Stop if we reached the goal
    if norm(current_state(1:3) - goal_pos) < 10
        disp('Goal Reached!');
        state_history = state_history(:, 1:i+1); % truncate arrays
        break;
    end
    
    % --- 3. Camera & EKF ---
    z = virtual_camera(current_state, obs_pos, noise_std);
    [x_est, P_est] = ekf_step(x_est, P_est, z, u_apply, V, dt, Q, R);
end

disp('Simulation Complete.');

%% 3. Visualization
Pn_hist = state_history(1, :);
Pe_hist = state_history(2, :);
Pd_hist = state_history(3, :);

figure('Name', 'Phase 4: Autonomous Obstacle Avoidance', 'Color', 'w');
plot3(Pe_hist, Pn_hist, -Pd_hist, 'b-', 'LineWidth', 2); hold on; grid on;
plot3(Pe_hist(1), Pn_hist(1), -Pd_hist(1), 'go', 'MarkerFaceColor', 'g');
plot3(goal_pos(2), goal_pos(1), -goal_pos(3), 'k*', 'MarkerSize', 10);

% Plot Obstacle
[X, Y, Z] = sphere(20); 
surf(obs_pos(2) + X*obs_radius, obs_pos(1) + Y*obs_radius, -(obs_pos(3) + Z*obs_radius), ...
    'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

xlabel('East (m)'); ylabel('North (m)'); zlabel('Height (m)');
title('Receding Horizon Autonomous Obstacle Avoidance');
legend('MAV Trajectory', 'Start', 'Goal', 'Obstacle');
view(3); axis equal;
