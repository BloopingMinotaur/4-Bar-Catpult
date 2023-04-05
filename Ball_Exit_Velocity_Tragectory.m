clear
clc
PKMS = importPKMS('KinematicDatafromPMKS.51.50.8.29.3.2023.txt');

g=9.81; %m/s2

%defines the variables takes fromt the PKMS simulation and saves the to
%discrete variables
theta_data = [PKMS.TimeSteps, PKMS.angle_output+pi/2];
omega_data = [PKMS.TimeSteps, PKMS.angVel_output];
alpha_data = [PKMS.TimeSteps, PKMS.angAccel_output];



% Initial conditions
p = PKMS.y_4(1);
pdot = sqrt(PKMS.Vx_4(1)^2+PKMS.Vy_4(1)^2);
p_doubledot = sqrt(PKMS.Ax_4(1)^2+PKMS.Ay_4(1)^2);
theta = PKMS.angle_output(1)+pi/2;
omega = PKMS.angVel_output(1);
alpha = PKMS.angAccel_output(1);

% Time span for simulation
tspan = [PKMS.TimeSteps(1), PKMS.TimeSteps(end)];

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
Ap_y = p .* alpha .* cos(theta) - p .* omega.^2 .* sin(theta) + pdot .* omega .* cos(theta) + pdot .* omega .* cos(theta) + p_doubledot .* sin(theta);

% Plot position, velocity, and acceleration
figure;
plot(t, Rp_x, 'r', t, Rp_y, 'b');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Rp_x', 'Rp_y');
title('Position vs. Time');

figure;
plot(t, Vp_x, 'r', t, Vp_y, 'b');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Vp_x', 'Vp_y');
title('Velocity vs. Time');

figure;
plot(t, Ap_x, 'r', t, Ap_y, 'b');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Ap_x', 'Ap_y');
title('Acceleration vs. Time');


function dXdt = system_dynamics(t, X, theta_data, omega_data, alpha_data)
    p = X(1);
    pdot = X(2);
    p_doubledot = X(3);

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

