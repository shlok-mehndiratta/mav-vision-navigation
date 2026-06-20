% MAIN_PHASE6_MONTE_CARLO: Statistical Analysis (Figure 7 reproduction)

clc; clear; close all;

%% 1. Monte Carlo Configuration
% In the paper, they ran 100 simulations per data point. 
% For testing purposes, we default to 3 so it doesn't take hours to run on a laptop!
% Change this to 100 later if you want academic-level statistical confidence.
num_sims = 3; 

% We will test environments with 2, 4, 6, and 8 obstacles.
num_obs_array = [2, 4, 6, 8];
avg_min_dist = zeros(1, length(num_obs_array));

disp('===================================================');
disp('  PHASE 6: MONTE CARLO STATISTICAL ANALYSIS');
disp('===================================================');
disp(['Running ', num2str(num_sims), ' simulations per density level.']);
disp('This might take a few minutes. Please wait...');

%% 2. Batch Execution Loop
for i = 1:length(num_obs_array)
    num_obs = num_obs_array(i);
    fprintf('\n-> Testing Density: %d Obstacles...\n', num_obs);
    
    min_dists_for_this_density = zeros(1, num_sims);
    
    for sim = 1:num_sims
        fprintf('   Sim %d/%d... ', sim, num_sims);
        
        % 1. Randomize Obstacle Positions in the flight corridor
        obs_positions = zeros(3, num_obs);
        obs_positions(1, :) = 200 + rand(1, num_obs) * 600; % North
        obs_positions(2, :) = 300 + rand(1, num_obs) * 600; % East
        obs_positions(3, :) = -30 - rand(1, num_obs) * 50;  % Down (Height)
        
        % 2. Run the isolated simulation and return the closest it ever got to a crash
        min_dist = run_single_sim(obs_positions);
        
        min_dists_for_this_density(sim) = min_dist;
        fprintf('Closest Call: %.2f m\n', min_dist);
    end
    
    % Average the closest calls for this density level
    avg_min_dist(i) = mean(min_dists_for_this_density);
end

disp('Monte Carlo Analysis Complete!');

%% 3. Plotting Figure 7
figure('Name', 'Paper Reproduction: Figure 7', 'Color', 'w');
plot(num_obs_array, avg_min_dist, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
grid on; hold on;
xlabel('Number of Obstacles');
ylabel('Average Minimum Distance (m)');
title('Algorithm Robustness vs. Obstacle Density');

% Draw a red line at 20m (the hard collision radius)
yline(20, 'r--', 'Collision Threshold (20m)', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');
ylim([0, max(avg_min_dist) + 10]);


%% ========================================================================
% LOCAL FUNCTION: Runs a single "headless" simulation without 3D plotting
% =========================================================================
function min_dist_run = run_single_sim(obs_positions)
    dt = 0.5; t_end = 150; time = 0:dt:t_end; V = 13;
    initial_state = [0; 100; -20; 0];
    goal_pos = [1000; 1100; -100];
    obs_radius = 20;
    
    num_obs = size(obs_positions, 2);
    current_state = initial_state;
    
    R_var = 0.0012; noise_std = [sqrt(R_var); sqrt(R_var)];
    Q = diag([0.00001, 0.0001, 0.0001]); R = diag([R_var, R_var]);
    x_est = zeros(3, num_obs); P_est = cell(1, num_obs);
    
    for j = 1:num_obs
        z_init = virtual_camera(current_state, obs_positions(:, j), noise_std);
        x_est(:, j) = [0.06; z_init(1); z_init(2)];
        P_est{j} = diag([0.03^2, R_var, R_var]);
    end
    
    m_horizon = 3; phi_max = deg2rad(30); theta_max = deg2rad(15);
    lb = repmat([-phi_max, -theta_max], 1, m_horizon);
    ub = repmat([phi_max,  theta_max], 1, m_horizon);
    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
    
    overall_min_dist = inf;
    
    for i = 1:(length(time)-1)
        U0 = zeros(1, 2*m_horizon);
        U_opt = fmincon(@(U) planner_cost_multi(U, current_state, x_est, V, dt, obs_radius, goal_pos, obs_positions), ...
                        U0, [], [], [], [], lb, ub, [], options);
        u_apply = [U_opt(1); U_opt(2)];
        
        current_state = current_state + mav_kinematics(0, current_state, u_apply, V)*dt;
        
        % Check true distance to all obstacles during this step
        for j = 1:num_obs
            dist = norm(current_state(1:3) - obs_positions(:, j));
            if dist < overall_min_dist
                overall_min_dist = dist;
            end
        end
        
        if norm(current_state(1:3) - goal_pos) < 30
            break; % Goal Reached
        end
        
        % Update all EKFs
        for j = 1:num_obs
            z = virtual_camera(current_state, obs_positions(:, j), noise_std);
            [x_est(:, j), P_est{j}] = ekf_step(x_est(:, j), P_est{j}, z, u_apply, V, dt, Q, R);
        end
    end
    
    min_dist_run = overall_min_dist;
end
