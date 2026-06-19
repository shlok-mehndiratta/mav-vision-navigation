% MAIN_PHASE3: Simulates MAV, Virtual Camera, and EKF State Estimation

clc; clear; close all;

%% 1. Initialization
dt = 0.1;           
t_end = 20;         
time = 0:dt:t_end;  
V = 13;             

initial_state = [120; 120; -60; 0]; 
state_history = zeros(4, length(time));
state_history(:, 1) = initial_state;
current_state = initial_state;

obs_pos = [300; 150; -60]; 
obs_radius = 20; 

% Camera Noise (Variance = 0.0012 rad^2, from Section V of the paper)
R_var = 0.0012;
noise_std = [sqrt(R_var); sqrt(R_var)]; 

%% --- NEW: EKF Initialization ---
% Process noise Q and Measurement noise R (Values exactly from paper)
Q = diag([0.00001, 0.0001, 0.0001]);
R = diag([R_var, R_var]);

% Paper initialization: tau_0 = 0.06, std_tau_0 = 0.03
tau_0 = 0.06;
P_est = diag([0.03^2, R_var, R_var]);

% Arrays to store EKF history for plotting
ekf_state_history = zeros(3, length(time)-1);
ekf_P_history = zeros(3, length(time)-1); % Store variances (diagonals of P)

% We will calculate true tau to compare our EKF against
true_tau_history = zeros(1, length(time)-1);

% First measurement setup
z_initial = virtual_camera(current_state, obs_pos, noise_std);
x_est = [tau_0; z_initial(1); z_initial(2)]; % [tau, eta, xi]

%% 2. Main Simulation Loop
for i = 1:(length(time) - 1)
    
    % MAV Control Input
    phi = deg2rad(15); 
    theta = deg2rad(5); 
    u = [phi; theta];
    
    % --- 1. MAV Moves ---
    state_dot = mav_kinematics(time(i), current_state, u, V);
    current_state = current_state + state_dot * dt;
    state_history(:, i+1) = current_state;
    
    % Calculate True Tau for our reference
    dist_3d = norm(obs_pos - current_state(1:3));
    true_tau_history(i) = V / dist_3d;
    
    % --- 2. Take a Picture ---
    z = virtual_camera(current_state, obs_pos, noise_std);
    
    % --- 3. Run EKF Step ---
    [x_est, P_est] = ekf_step(x_est, P_est, z, u, V, dt, Q, R);
    
    % Save EKF data
    ekf_state_history(:, i) = x_est;
    ekf_P_history(:, i) = diag(P_est); % Save the variances
end

%% 3. Visualization (Recreating Figure 6 from the paper)
% Tracking Errors
tau_error = ekf_state_history(1, :) - true_tau_history;

% 2-Sigma Bounds (95% Confidence Interval)
sigma_tau = sqrt(ekf_P_history(1, :));
bound_tau = 2 * sigma_tau;

figure('Name', 'Phase 3: EKF Inverse TTC Tracking Error', 'Color', 'w');
plot(time(1:end-1), tau_error, 'b-', 'LineWidth', 1.5); hold on;
plot(time(1:end-1), bound_tau, 'r--', 'LineWidth', 1.5);
plot(time(1:end-1), -bound_tau, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Inverse TTC Tracking Error (1/s)');
title('EKF Performance: Inverse TTC Error with \pm2\sigma Bounds');
legend('Tracking Error', '\pm2\sigma Bounds');
