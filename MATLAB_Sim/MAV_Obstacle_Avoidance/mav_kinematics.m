function state_dot = mav_kinematics(t, state, u, V)
    % MAV_KINEMATICS calculates the derivatives of the MAV state
    % Inputs:
    %   t     - Current time (required by MATLAB ode solvers)
    %   state - A column vector [Pn; Pe; Pd; psi]
    %   u     - Control input vector [roll (phi); pitch (theta)]
    %   V     - Constant ground speed

    g = 9.81; % Gravity (m/s^2)
    
    % Extract current states
    Pn = state(1);
    Pe = state(2);
    Pd = state(3);
    psi = state(4);
    
    % Extract control inputs
    phi = u(1);
    theta = u(2);
    
    % Kinematic equations
    Pn_dot = V * cos(theta) * cos(psi);
    Pe_dot = V * cos(theta) * sin(psi);
    Pd_dot = -V * sin(theta);
    psi_dot = (g / V) * tan(phi);
    
    % Return the derivatives as a column vector
    state_dot = [Pn_dot; Pe_dot; Pd_dot; psi_dot];
end
