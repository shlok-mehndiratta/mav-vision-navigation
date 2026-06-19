function J = planner_cost(U_vec, current_state, ekf_state, V, dt, obs_radius, goal_pos)
    % PLANNER_COST calculates the penalty for a sequence of control inputs
    %
    % Inputs:
    %   U_vec - Array of future controls [phi_1, theta_1, phi_2, theta_2, ...]
    %   current_state, ekf_state, V, dt, obs_radius, goal_pos - current system variables

    m_steps = length(U_vec) / 2; % Horizon length
    J = 0; % Initial cost
    g = 9.81;
    
    % Weights from the paper (Section V)
    a1 = 1; a2 = 1; a3 = 1; bi = 10;
    
    sim_state = current_state;
    sim_ekf = ekf_state;
    
    for k = 1:m_steps
        phi = U_vec(2*k - 1);
        theta = U_vec(2*k);
        
        % 1. Predict Future Physical State
        u_k = [phi; theta];
        sim_state = sim_state + mav_kinematics(0, sim_state, u_k, V) * dt;
        
        % 2. Predict Future EKF State
        tau = sim_ekf(1); eta = sim_ekf(2); xi = sim_ekf(3);
        psi_dot = (g/V) * tan(phi);
        
        tau_dot = tau^2 * cos(theta) * cos(eta) * cos(xi) + tau^2 * sin(theta) * sin(xi);
        eta_dot = (tau * cos(theta) * sin(eta)) / cos(xi) - psi_dot;
        xi_dot  = tau * cos(theta) * cos(eta) * sin(xi) - tau * sin(theta) * cos(xi);
        
        sim_ekf = sim_ekf + [tau_dot; eta_dot; xi_dot] * dt;
        
        % 3. Goal Tracking Cost (Heading towards destination)
        % Calculate relative angle to the goal
        R_yaw = [cos(sim_state(4)), sin(sim_state(4)), 0;
                -sin(sim_state(4)), cos(sim_state(4)), 0; 0, 0, 1];
        rel_goal = R_yaw * (goal_pos - sim_state(1:3));
        
        dist_goal = norm(rel_goal);
        tau_g = V / dist_goal;
        eta_g = atan2(rel_goal(2), rel_goal(1));
        xi_g = asin(rel_goal(3) / dist_goal);
        
        goal_cost = (a1 / (tau_g^2 + 1e-6)) + a2 * eta_g^2 + a3 * xi_g^2;
        
        % 4. Observability / Collision Avoidance Cost
        % The paper proves the system is "Unobservable" (and on a collision course)
        % if these specific mathematical terms hit zero:
        O31 = (cos(theta)*sin(eta)) / cos(xi);
        O41 = cos(theta)*cos(eta)*sin(xi) - sin(theta)*cos(xi);
        O51 = (4*tau*cos(theta)^2 * sin(eta)*cos(eta))/(cos(xi)^2) - (psi_dot*cos(theta)*cos(eta))/cos(xi);
        
        % The penalty is the inverse of their magnitude (blows up if flying straight at obstacle)
        observability_penalty = bi / (O31^2 + O41^2 + O51^2 + 1e-6);
        
        % Hard collision penalty if predicted distance < safe radius
        if (V / sim_ekf(1)) < (obs_radius * 1.5)
            observability_penalty = observability_penalty + 10000; 
        end
        
        % Accumulate total cost for this step
        J = J + goal_cost + observability_penalty;
    end
end
