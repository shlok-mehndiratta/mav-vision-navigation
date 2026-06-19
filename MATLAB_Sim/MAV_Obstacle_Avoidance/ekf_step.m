function [x_new, P_new] = ekf_step(x_est, P_est, z, u, V, dt, Q, R)
    % EKF_STEP performs one Predict and Update step for the bearing-only EKF
    %
    % Inputs:
    %   x_est - Current state estimate [tau; eta; xi]
    %   P_est - Current covariance matrix (3x3)
    %   z     - Noisy camera measurement [eta_meas; xi_meas]
    %   u     - Control inputs [phi (roll); theta (pitch)]
    %   V     - MAV speed
    %   dt    - Time step
    %   Q     - Process noise covariance
    %   R     - Measurement noise covariance
    
    tau = x_est(1);
    eta = x_est(2);
    xi  = x_est(3);
    
    phi   = u(1);
    theta = u(2);
    g     = 9.81;
    
    psi_dot = (g / V) * tan(phi);
    
    %% --- PREDICT STEP ---
    % 1. Non-linear state propagation (Equations 4 from paper)
    tau_dot = tau^2 * cos(theta) * cos(eta) * cos(xi) + tau^2 * sin(theta) * sin(xi);
    eta_dot = (tau * cos(theta) * sin(eta)) / cos(xi) - psi_dot;
    xi_dot  = tau * cos(theta) * cos(eta) * sin(xi) - tau * sin(theta) * cos(xi);
    
    % Euler integration for the state prediction
    x_pred = x_est + [tau_dot; eta_dot; xi_dot] * dt;
    
    % 2. Calculate the Jacobian Matrix (A) for the covariance prediction
    A = zeros(3,3);
    
    % d(tau_dot) / d(state)
    A(1,1) = 2*tau*(cos(theta)*cos(eta)*cos(xi) + sin(theta)*sin(xi));
    A(1,2) = -tau^2 * cos(theta)*sin(eta)*cos(xi);
    A(1,3) = -tau^2 * cos(theta)*cos(eta)*sin(xi) + tau^2 * sin(theta)*cos(xi);
    
    % d(eta_dot) / d(state)
    A(2,1) = (cos(theta)*sin(eta)) / cos(xi);
    A(2,2) = (tau*cos(theta)*cos(eta)) / cos(xi);
    A(2,3) = (tau*cos(theta)*sin(eta)*sin(xi)) / (cos(xi)^2);
    
    % d(xi_dot) / d(state)
    A(3,1) = cos(theta)*cos(eta)*sin(xi) - sin(theta)*cos(xi);
    A(3,2) = -tau*cos(theta)*sin(eta)*sin(xi);
    A(3,3) = tau*cos(theta)*cos(eta)*cos(xi) + tau*sin(theta)*sin(xi);
    
    % Discretize the A matrix (Phi = I + A*dt)
    Phi = eye(3) + A * dt;
    
    % Predict Covariance
    P_pred = Phi * P_est * Phi' + Q * dt;
    
    %% --- UPDATE STEP ---
    % Measurement matrix H (We only measure state 2 and 3: eta and xi)
    H = [0, 1, 0; 
         0, 0, 1];
     
    % Measurement residual (difference between measured and predicted)
    y = z - H * x_pred;
    
    % Handle angle wrap-around for the residual
    y(1) = wrapToPi(y(1));
    y(2) = wrapToPi(y(2));
    
    % Kalman Gain
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    % Final State and Covariance Update
    x_new = x_pred + K * y;
    P_new = (eye(3) - K * H) * P_pred;
end
