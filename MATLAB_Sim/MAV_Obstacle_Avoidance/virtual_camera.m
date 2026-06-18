function z = virtual_camera(mav_state, obs_pos, noise_std)
    % VIRTUAL_CAMERA calculates the noisy bearing measurements
    %
    % Inputs:
    %   mav_state - [Pn; Pe; Pd; psi] of the MAV
    %   obs_pos   - [On; Oe; Od] of the Obstacle
    %   noise_std - Standard deviation for the measurement noise [std_eta; std_xi]
    %
    % Output:
    %   z - Measurement vector [azimuth (eta); elevation (xi)] in radians

    % Extract MAV state
    Pn = mav_state(1);
    Pe = mav_state(2);
    Pd = mav_state(3);
    psi = mav_state(4);

    % Calculate relative position in the Inertial (NED) frame
    delta_N = obs_pos(1) - Pn;
    delta_E = obs_pos(2) - Pe;
    delta_D = obs_pos(3) - Pd;

    % Rotation matrix to convert from Inertial to Local-Level frame
    % (Rotates around the Z-Down axis by the heading angle psi)
    R_yaw = [cos(psi),  sin(psi), 0;
            -sin(psi), cos(psi), 0;
             0,         0,        1];
         
    relative_loc = R_yaw * [delta_N; delta_E; delta_D];
    x_loc = relative_loc(1); % Nose direction
    y_loc = relative_loc(2); % Right wing direction
    z_loc = relative_loc(3); % Down direction

    % Calculate true Azimuth (eta) and Elevation (xi)
    % Azimuth: Right-handed rotation about z-axis
    eta_true = atan2(y_loc, x_loc);
    
    % Elevation: Right-handed rotation about y-axis (Positive is looking DOWN)
    r_3d = norm(relative_loc);
    xi_true = asin(z_loc / r_3d);

    % Add Gaussian Noise to simulate a real camera
    % randn() generates a random number from a standard normal distribution (mean=0, std=1)
    eta_noisy = eta_true + noise_std(1) * randn();
    xi_noisy  = xi_true  + noise_std(2) * randn();

    % Return the noisy measurements
    z = [eta_noisy; xi_noisy];
end
