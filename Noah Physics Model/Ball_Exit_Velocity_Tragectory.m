clear
clc
close all

PKMS = importPKMS('KinematicDatafromPMKS.15rpm.txt');
m = 0.001; % kg
g =9.81; % m/s2

%defines the variables takes fromt the PKMS simulation and saves the to
%discrete variables
theta_data = [PKMS.TimeSteps, PKMS.angle_output+pi/2];
omega_data = [PKMS.TimeSteps, -1*PKMS.angVel_output];
alpha_data = [PKMS.TimeSteps, PKMS.angAccel_output];

% Finds the index where the robot arm starts rotating CW (
[min_Theta_Val, startIndex] = min(PKMS.angle_output);

% Initial conditions
p = PKMS.y_4(startIndex);
pdot = sqrt(PKMS.Vx_4(startIndex)^2+PKMS.Vy_4(startIndex)^2);
p_doubledot = sqrt(PKMS.Ax_4(startIndex)^2+PKMS.Ay_4(startIndex)^2);
theta = PKMS.angle_output(startIndex)+pi/2;
omega = PKMS.angVel_output(startIndex);
alpha = PKMS.angAccel_output(startIndex);

% Time span for simulation
tspan = [PKMS.TimeSteps(startIndex), PKMS.TimeSteps(end)];

% Initial state vector
X0 = [p; pdot; p_doubledot];

% Set up the event function
options = odeset('Events', @event_function);

% Solve the system using ode45
[t, X] = ode45(@(t, X) system_dynamics(t, X, theta_data, omega_data, alpha_data), tspan, X0, options);

% Extract and compute position, velocity, and acceleration
p = X(:, 1); 
pdot = X(:, 2);
p_doubledot = X(:, 3);


Rp_x = p .* cos(theta);
Rp_y = p .* sin(theta);

Vp_x = -p .* omega .* sin(theta) + pdot .* cos(theta);
Vp_y = p .* omega .* cos(theta) + pdot .* sin(theta);

Ap_x = -p .* alpha .* sin(theta) - p .* omega.^2 .* cos(theta) - pdot .* omega .* sin(theta) - pdot .* omega .* sin(theta) + p_doubledot .* cos(theta);
Ap_y = p .* alpha .* cos(theta) - p .* omega.^2 .* sin(theta) + pdot .* omega .* cos(theta) + pdot .* omega .* cos(theta) + p_doubledot .* sin(theta) - m * g;

% Extract the final state values
p_final = X(end, 1);
pdot_final = X(end, 2);
p_doubledot_final = X(end, 3);
theta_final = interp1(theta_data(:,1), theta_data(:,2), t(end)) % radians
theta_final_deg = rad2deg(theta_final)

% Calculate the trajectory after p exceeds 80
[trajectory_time, trajectory_data] = trajectory_after_p_exceeds_80(t(end), Rp_x(end), Rp_y(end), Vp_x(end), Vp_y(end));

x_trajectory = trajectory_data(:, 1);
y_trajectory = trajectory_data(:, 2);

total_Rp_x = [Rp_x; x_trajectory];
total_Rp_y = [Rp_y; y_trajectory];

total_time = [t; trajectory_time'];

% Plot position, velocity, and acceleration
figure;
hold on
plot(total_Rp_x, total_Rp_y,'b');
plot(x_trajectory,y_trajectory,'r');
hold off
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Total Trajectory of p');
legend('Ball In Basket', 'Ball Released From basket','location','best');

figure;
plot(total_time, total_Rp_x);
xlabel('Time (s)');
ylabel('X Position (m)');
title('X Position vs. Time');

figure;
plot(total_time, total_Rp_y);
xlabel('Time (s)');
ylabel('Y Position (m)');
title('Y Position vs. Time')

figure;
plot(t, Rp_x, 'r', t, Rp_y, 'b');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Rp_x', 'Rp_y','location','best');
title('Position vs. Time');

figure;
plot(t, Vp_x, 'r', t, Vp_y, 'b');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Vp_x', 'Vp_y','location','best');
title('Velocity vs. Time');

figure;
plot(t, Ap_x, 'r', t, Ap_y, 'b');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Ap_x', 'Ap_y','location','best');
title('Acceleration vs. Time');


function dXdt = system_dynamics(t, X, theta_data, omega_data, alpha_data)
    p = X(1);
    pdot = X(2);
    p_doubledot = X(3);
    
    % Ensure p is not less than the minimum value (60)
    p = max(p, 60);

    % Interpolate theta, omega, and alpha at the current simulation time step
    theta = interp1(theta_data(:,1), theta_data(:,2), t);
    omega = interp1(omega_data(:,1), omega_data(:,2), t);
    alpha = interp1(alpha_data(:,1), alpha_data(:,2), t);

    % System matrix A
    A = [0, 1, 0;
         0, 0, 1;
         0, 0, 0];
     
    % Input matrix B
    B = [0;
         0;
         1];
         
    dXdt = A * X + B * alpha;
end


function [value, isterminal, direction] = event_function(t, X)
    p = X(1);
    value = 80 - p; % Trigger when p > 80
    isterminal = 1; % Stop the solver when the event is triggered
    direction = []; % Trigger the event in any direction
end

% Function to calculate the trajectory after p exceeds 80
function [t_trajectory, trajectory_data] = trajectory_after_p_exceeds_80(t_start, Rp_x_final, Rp_y_final, Vp_x_final, Vp_y_final)
    g = 9.81; % Acceleration due to gravity (m/s^2)
    
    % Use final values of Rp_x, Rp_y, Vp_x, and Vp_y as initial conditions
    x_initial = Rp_x_final;
    y_initial = Rp_y_final;
    vx_initial = Vp_x_final;
    vy_initial = Vp_y_final;

    % Time span for the trajectory calculation
    t_end = t_start + 100; % Adjust this value as needed
    t_trajectory = linspace(t_start, t_end, 1000);

    % Initialize trajectory arrays
    x_trajectory = zeros(size(t_trajectory));
    y_trajectory = zeros(size(t_trajectory));

    % Calculate the trajectory and check for termination condition
    for i = 1:length(t_trajectory)
        t_curr = t_trajectory(i);
        x_trajectory(i) = x_initial + vx_initial * (t_curr - t_start);
        y_trajectory(i) = y_initial + vy_initial * (t_curr - t_start) - 0.5 * g * (t_curr - t_start).^2;

        % Terminate trajectory calculation if y_trajectory is equal to or less than zero
        if y_trajectory(i) <= 0
            t_trajectory = t_trajectory(1:i);
            x_trajectory = x_trajectory(1:i);
            y_trajectory = y_trajectory(1:i);
            break;
        end
    end

    % Combine the trajectory data
    trajectory_data = [x_trajectory', y_trajectory'];
end



