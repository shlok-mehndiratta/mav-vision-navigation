function J = planner_cost_multi(U_vec, current_state, ekf_states, V, dt, obs_radius, goal_pos, obs_positions)
    % PLANNER_COST_MULTI handles multiple obstacles for the optimization
    
    m_steps = length(U_vec) / 2;
    J = 0;
    g = 9.81;
    
    a1 = 5000; a2 = 100; a3 = 100; bi = 5000;
    
    num_obs = size(ekf_states, 2);
    
    sim_state = current_state;
    sim_ekf = ekf_states; % 3xN matrix of obstacle states
    
    for k = 1:m_steps
        phi = U_vec(2*k - 1);
        theta = U_vec(2*k);
        u_k = [phi; theta];
        
        sim_state = sim_state + mav_kinematics(0, sim_state, u_k, V) * dt;
        psi_dot = (g/V) * tan(phi);
        
        % Goal Tracking Cost
        R_yaw = [cos(sim_state(4)),  sin(sim_state(4)), 0;
                -sin(sim_state(4)), cos(sim_state(4)), 0; 
                 0,                  0,                 1];
        rel_goal = R_yaw * (goal_pos - sim_state(1:3));
        
        dist_goal = norm(rel_goal);
        tau_g = V / dist_goal;
        eta_g = atan2(rel_goal(2), rel_goal(1));
        xi_g = asin(rel_goal(3) / dist_goal);
        
        goal_cost = (a1 / (tau_g^2 + 1e-6)) + a2 * eta_g^2 + a3 * xi_g^2;
        J = J + goal_cost;
        
        % Loop over all obstacles to accumulate penalties
        for obs_idx = 1:num_obs
            tau = sim_ekf(1, obs_idx);
            eta = sim_ekf(2, obs_idx);
            xi  = sim_ekf(3, obs_idx);
            
            % Predict Future EKF State
            tau_dot = tau^2 * cos(theta) * cos(eta) * cos(xi) + tau^2 * sin(theta) * sin(xi);
            eta_dot = (tau * cos(theta) * sin(eta)) / cos(xi) - psi_dot;
            xi_dot  = tau * cos(theta) * cos(eta) * sin(xi) - tau * sin(theta) * cos(xi);
            
            sim_ekf(:, obs_idx) = sim_ekf(:, obs_idx) + [tau_dot; eta_dot; xi_dot] * dt;
            
            % Observability / Collision Cost
            if abs(eta) < pi/2 && abs(xi) < pi/2
                O31 = (cos(theta)*sin(eta)) / cos(xi);
                O41 = cos(theta)*cos(eta)*sin(xi) - sin(theta)*cos(xi);
                O51 = (4*tau*cos(theta)^2 * sin(eta)*cos(eta))/(cos(xi)^2) - (psi_dot*cos(theta)*cos(eta))/cos(xi);
                
                obs_penalty = bi / (O31^2 + O41^2 + O51^2 + 1e-6);
                
                % Hard collision penalty using True Distance to overcome EKF initialization lag
                true_dist = norm(sim_state(1:3) - obs_positions(:, obs_idx));
                
                % Add a soft repelling force field so it swerves EARLY instead of waiting
                % until it's 30 meters away (which is too late to turn at 13m/s)
                soft_penalty = 5e8 / (true_dist^2 + 1e-6);
                
                if true_dist < (obs_radius * 1.5)
                    obs_penalty = obs_penalty + soft_penalty + 1e8;
                else
                    obs_penalty = obs_penalty + soft_penalty;
                end
                
                J = J + obs_penalty;
            end
        end
    end
end
