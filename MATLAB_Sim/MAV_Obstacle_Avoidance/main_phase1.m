% MAIN_PHASE1: Simulates and visualizes the MAV kinematics

%% 1. Initialization
clc;        % Clears the command window
clear;      % Clears the workspace memory
close all;  % Closes all open figures

% Simulation parameters
dt = 0.1;           % Time step of 0.1 seconds
t_end = 20;         % Simulate for 20 seconds
time = 0:dt:t_end;  % Create an array of time values

V = 13;             % MAV speed in m/s (as specified in the paper)

% Initial State: [North; East; Down; Heading]
% Down is negative in aviation, so a height of 60m is Pd = -60
initial_state = [120; 120; -60; 0]; 

% We will store the history of the states here for plotting
% Pre-allocate an array of zeros for speed
state_history = zeros(4, length(time));
state_history(:, 1) = initial_state;

% Current state
current_state = initial_state;

%% 2. Main Simulation Loop
for i = 1:(length(time) - 1)
    
    % For Phase 1, let's create a test maneuver:
    % Bank right (roll = 15 deg) and pitch up (pitch = 5 deg)
    phi = deg2rad(15); 
    theta = deg2rad(5); 
    u = [phi; theta];
    
    % Calculate derivatives using our function
    state_dot = mav_kinematics(time(i), current_state, u, V);
    
    % Euler Integration: New State = Old State + (Rate of Change * dt)
    current_state = current_state + state_dot * dt;
    
    % Save state to history for plotting
    state_history(:, i+1) = current_state;
end

%% 3. Visualization
% Extract coordinates for plotting
Pn_hist = state_history(1, :);
Pe_hist = state_history(2, :);
Pd_hist = state_history(3, :);

% Create a 3D Plot
figure('Name', 'MAV Trajectory Phase 1', 'Color', 'w');
plot3(Pe_hist, Pn_hist, -Pd_hist, 'b-', 'LineWidth', 2);
hold on;
grid on;

% Mark the start and end points
plot3(Pe_hist(1), Pn_hist(1), -Pd_hist(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
plot3(Pe_hist(end), Pn_hist(end), -Pd_hist(end), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Add labels (Note: NED convention means we plot East on X, North on Y, and -Down on Z for height)
xlabel('East (m)');
ylabel('North (m)');
zlabel('Height (m)');
title('MAV 3D Flight Path');
legend('Trajectory', 'Start', 'End');
view(3); % Set 3D view angle
