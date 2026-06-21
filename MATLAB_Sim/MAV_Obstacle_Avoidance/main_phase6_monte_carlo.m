% MAIN_PHASE6_MONTE_CARLO: Statistical Analysis (Figures 7 & 8 reproduction)
% This script runs multiple headless simulations with varying parameters
% to reproduce the academic statistical robustness curves.

clc; clear; close all;

%% 1. Monte Carlo Configuration
% In the paper, they ran 100 simulations per data point. 
num_sims = 2; 

% Test Array 1: Minimum Distance (Figure 7)
min_dist_array = [5, 10, 15, 20, 25, 30];

% Test Array 2: Measurement Noise in Degrees (Figure 8)
sigma_m_array = [0, 5, 10, 15, 20];

disp('===================================================');
disp('  PHASE 6: MONTE CARLO STATISTICAL ANALYSIS');
disp('===================================================');
disp(['Running ', num2str(num_sims), ' simulations per data point.']);
disp('This might take a few minutes. Please wait...');

%% =========================================================================
% FIGURE 7 EXPERIMENT: Varying Minimum Distance between Obstacles
% ==========================================================================
disp(' ');
disp('--- Running Figure 7 Experiment (Varying Min Distance) ---');
fig7_collisions_noisy = zeros(1, length(min_dist_array));
fig7_goal_noisy = zeros(1, length(min_dist_array));

fig7_collisions_perfect = zeros(1, length(min_dist_array));
fig7_goal_perfect = zeros(1, length(min_dist_array));

% Fixed noise for Figure 7 is 2 degrees standard deviation
noise_std_fixed = deg2rad(2) * ones(2, 1); 

for i = 1:length(min_dist_array)
    min_dist = min_dist_array(i);
    fprintf('Testing Min Distance: %d meters...\n', min_dist);
    
    col_noisy_sum = 0; goal_noisy_sum = 0;
    col_perf_sum = 0;  goal_perf_sum = 0;
    
    for sim = 1:num_sims
        % 1. Generate Environment until no more obstacles can be added
        obs_positions = generate_environment(min_dist);
        
        % 2. Run simulation WITH measurement noise (Solid Line in Fig 7)
        [c_noisy, g_noisy] = run_single_sim(obs_positions, noise_std_fixed, false);
        col_noisy_sum = col_noisy_sum + c_noisy;
        goal_noisy_sum = goal_noisy_sum + g_noisy;
        
        % 3. Run simulation WITHOUT noise (Perfectly known - Dashed Line)
        [c_perf, g_perf] = run_single_sim(obs_positions, [0;0], true);
        col_perf_sum = col_perf_sum + c_perf;
        goal_perf_sum = goal_perf_sum + g_perf;
    end
    
    fig7_collisions_noisy(i) = col_noisy_sum / num_sims;
    fig7_goal_noisy(i) = (goal_noisy_sum / num_sims) * 100;
    
    fig7_collisions_perfect(i) = col_perf_sum / num_sims;
    fig7_goal_perfect(i) = (goal_perf_sum / num_sims) * 100;
end

%% =========================================================================
% FIGURE 8 EXPERIMENT: Varying Measurement Noise
% ==========================================================================
disp(' ');
disp('--- Running Figure 8 Experiment (Varying Noise) ---');
fig8_collisions = zeros(1, length(sigma_m_array));
fig8_goal = zeros(1, length(sigma_m_array));

% Fixed minimum distance for Figure 8 is 20 meters
min_dist_fixed = 20;

for i = 1:length(sigma_m_array)
    sigma_deg = sigma_m_array(i);
    fprintf('Testing Noise Level: %d degrees...\n', sigma_deg);
    
    noise_std_test = deg2rad(sigma_deg) * ones(2, 1);
    
    col_sum = 0; goal_sum = 0;
    for sim = 1:num_sims
        obs_positions = generate_environment(min_dist_fixed);
        
        [c, g] = run_single_sim(obs_positions, noise_std_test, false);
        col_sum = col_sum + c;
        goal_sum = goal_sum + g;
    end
    
    fig8_collisions(i) = col_sum / num_sims;
    fig8_goal(i) = (goal_sum / num_sims) * 100;
end

disp('Monte Carlo Analysis Complete!');

%% =========================================================================
% PLOTTING RESULTS
% ==========================================================================

% Plot Figure 7
figure('Name', 'Paper Reproduction: Figure 7', 'Color', 'w', 'Position', [100, 100, 1000, 400]);
subplot(1, 2, 1);
plot(min_dist_array, fig7_collisions_noisy, 'b-', 'LineWidth', 2); hold on; grid on;
plot(min_dist_array, fig7_collisions_perfect, 'r--', 'LineWidth', 2);
xlabel('Minimum distance (m)'); ylabel('Number of collisions');
title('(a)'); legend('Noisy (2 deg)', 'Perfectly Known');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');

subplot(1, 2, 2);
plot(min_dist_array, fig7_goal_noisy, 'b-', 'LineWidth', 2); hold on; grid on;
plot(min_dist_array, fig7_goal_perfect, 'r--', 'LineWidth', 2);
xlabel('Minimum distance (m)'); ylabel('Percentage of goal reaching');
title('(b)'); ylim([0 105]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');

% Plot Figure 8
figure('Name', 'Paper Reproduction: Figure 8', 'Color', 'w', 'Position', [150, 150, 1000, 400]);
subplot(1, 2, 1);
plot(sigma_m_array, fig8_collisions, 'k-o', 'LineWidth', 2, 'MarkerFaceColor', 'k'); grid on;
xlabel('\sigma_m (degree)'); ylabel('Number of collisions');
title('(a)');
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');

subplot(1, 2, 2);
plot(sigma_m_array, fig8_goal, 'k-o', 'LineWidth', 2, 'MarkerFaceColor', 'k'); grid on;
xlabel('\sigma_m (degree)'); ylabel('Percentage of goal reaching');
title('(b)'); ylim([0 105]);
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k');

%% ========================================================================
% LOCAL FUNCTIONS
% =========================================================================
function obs_positions = generate_environment(min_dist)
    % Uniform distribution over cubic area (100,100,-20) to (600,600,-100)
    % Paper: "each obstacle is added... until no more obstacle can be added."
    obs_positions = [];
    max_consecutive_failures = 500; % Stop trying after 500 failed placements
    failures = 0;
    
    while failures < max_consecutive_failures
        pos = [100 + rand * 500;  % North
               100 + rand * 500;  % East
               -20 - rand * 80];  % Down
               
        if isempty(obs_positions)
            obs_positions = pos;
        else
            % Calculate 2D distance to all existing obstacles
            dists = sqrt((obs_positions(1,:) - pos(1)).^2 + (obs_positions(2,:) - pos(2)).^2);
            
            % Ensure it doesn't spawn exactly on the Start (120, 120) or Goal (580, 580)
            dist_to_start = norm(pos(1:2) - [120; 120]);
            dist_to_goal = norm(pos(1:2) - [580; 580]);
            
            if all(dists >= min_dist) && dist_to_start > 30 && dist_to_goal > 30
                obs_positions = [obs_positions, pos];
                failures = 0; % Reset failures because we successfully placed one
            else
                failures = failures + 1;
            end
        end
    end
end

function [num_collisions, reached_goal] = run_single_sim(obs_positions, noise_std, use_true_state)
    dt = 0.5; t_end = 100; time = 0:dt:t_end; V = 13;
    
    % As stated in Section V.B of the paper:
    initial_state = [120; 120; -60; 0];
    goal_pos = [580; 580; -60];
    obs_radius = 20;
    
    num_obs = size(obs_positions, 2);
    current_state = initial_state;
    
    Q = diag([0.00001, 0.0001, 0.0001]); 
    R = diag([noise_std(1)^2 + 1e-8, noise_std(2)^2 + 1e-8]);
    
    x_est = zeros(3, num_obs); P_est = cell(1, num_obs);
    
    for j = 1:num_obs
        if use_true_state
            % Inject perfect truth directly bypassing camera and EKF
            rel_pos = obs_positions(:, j) - current_state(1:3);
            dist = norm(rel_pos);
            x_est(:, j) = [V / dist; atan2(rel_pos(2), rel_pos(1)); asin(rel_pos(3) / dist)];
            P_est{j} = eye(3) * 1e-6; % Tiny covariance
        else
            z_init = virtual_camera(current_state, obs_positions(:, j), noise_std);
            x_est(:, j) = [0.06; z_init(1); z_init(2)];
            P_est{j} = diag([0.03^2, R(1,1), R(2,2)]);
        end
    end
    
    m_horizon = 12; % Paper used exactly a 6-second look-ahead horizon (12 steps * 0.5 dt = 6s)
    phi_max = deg2rad(30); theta_max = deg2rad(15);
    lb = repmat([-phi_max, -theta_max], 1, m_horizon);
    ub = repmat([phi_max,  theta_max], 1, m_horizon);
    options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
    
    num_collisions = 0;
    hit_flags = false(1, num_obs);
    reached_goal = false;
    
    for i = 1:(length(time)-1)
        U0 = zeros(1, 2*m_horizon);
        % Pass x_est to the planner
        U_opt = fmincon(@(U) planner_cost_multi(U, current_state, x_est, V, dt, obs_radius, goal_pos, obs_positions), ...
                        U0, [], [], [], [], lb, ub, [], options);
        u_apply = [U_opt(1); U_opt(2)];
        
        current_state = current_state + mav_kinematics(0, current_state, u_apply, V)*dt;
        
        % Collision checking
        for j = 1:num_obs
            dist = norm(current_state(1:3) - obs_positions(:, j));
            if dist < obs_radius && ~hit_flags(j)
                num_collisions = num_collisions + 1;
                hit_flags(j) = true; % Mark as hit so we don't count it twice
            end
        end
        
        % Goal checking
        if norm(current_state(1:3) - goal_pos) < 30
            if num_collisions == 0
                reached_goal = true;
            end
            break; % Reached goal proximity
        end
        
        % Update all EKFs
        for j = 1:num_obs
            if use_true_state
                % Update perfect truth
                R_yaw = [cos(current_state(4)), sin(current_state(4)), 0;
                        -sin(current_state(4)), cos(current_state(4)), 0; 
                         0, 0, 1];
                rel_local = R_yaw * (obs_positions(:, j) - current_state(1:3));
                dist = norm(rel_local);
                x_est(:, j) = [V / dist; atan2(rel_local(2), rel_local(1)); asin(rel_local(3) / dist)];
            else
                z = virtual_camera(current_state, obs_positions(:, j), noise_std);
                [x_est(:, j), P_est{j}] = ekf_step(x_est(:, j), P_est{j}, z, u_apply, V, dt, Q, R);
            end
        end
    end
end
