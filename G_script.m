% Clean the environment for fresh start
clearvars;
close all;

% System parameters
massBase = 1000; % Cart mass (kg)
massPend1 = 100; % First pendulum mass (kg)
massPend2 = 100; % Second pendulum mass (kg)
lengthPend1 = 20; % First pendulum length (m)
lengthPend2 = 10; % Second pendulum length (m)
accelGravity = 9.81; % Acceleration due to gravity (m/s^2)

% State-space representation for the linearized version
A_lin=[0 1 0 0 0 0; 
    0 0 -(accelGravity*massPend1)/massBase 0 -(accelGravity*massPend2)/massBase 0;
    0 0 0 1 0 0;
    0 0 -((massBase+massPend1)*accelGravity)/(massBase*lengthPend1) 0 -(massPend2*accelGravity)/(massBase*lengthPend1) 0;
    0 0 0 0 0 1;
    0 0 -(massPend1*accelGravity)/(massBase*lengthPend2) 0 -(accelGravity*(massBase+massPend2))/(massBase*lengthPend2) 0]; 
B_lin=[0; 1/massBase; 0; 1/(massBase*lengthPend1); 0; 1/(massBase*lengthPend2)];
C_lin = [1 0 0 0 0 0];

% LQR controller configuration
Q_lqr = diag([1, 1, 1, 1, 1, 1]); % State weights
R_lqr = 1e-5; % Control effort weight
K_lqr = lqr(A_lin, B_lin, Q_lqr, R_lqr);

% Kalman Filter setup
processNoise = 0.1*eye(6); % Process noise covariance matrix
measurementNoise = 0.1; % Measurement noise variance
[estimatorGain, ~, ~] = lqe(A_lin, eye(6), C_lin, processNoise, measurementNoise);

% Simulation initial conditions
initialStates = [10; 0; 20; 0; 30; 0]; % Starting with pendula at small angles

% Define simulation duration
timeStart = 0;
timeEnd = 20;
timeSpanSim = [timeStart timeEnd];

% Solving ODE using MATLAB's solver
[timeSol, stateSol] = ode45(@(t, x) dynamicSystem(t, x, K_lqr, estimatorGain, A_lin, B_lin, C_lin, massBase, massPend1, massPend2, lengthPend1, lengthPend2, accelGravity), timeSpanSim, initialStates);

% Plot the response of states
figure('Name','System State Responses','NumberTitle','off');
stateLabels = {'Cart Position', 'Cart Velocity', 'Pendulum 1 Angle', 'Pendulum 1 Angular Velocity', 'Pendulum 2 Angle', 'Pendulum 2 Angular Velocity'};
for idx = 1:6
    subplot(6, 1, idx);
    plot(timeSol, stateSol(:, idx), 'LineWidth', 1.5);
    title(['Response of ', stateLabels{idx}]);
    xlabel('Time [s]');
    ylabel(['State ', num2str(idx)]);
    grid on;
end

% Nested function for dynamic system equations
function dx = dynamicSystem(t, x, K, L, A, B, C, M, m1, m2, l1, l2, g)
    y_meas = C * x; % Measurement from the first state
    x_est = A * x + B * (-K * x) + L * (y_meas - C * x); % Estimated states with LQG
    
    % Control action based on state estimates
    u_ctrl = -K * x_est;
    
    % Nonlinear system dynamics
    dx = zeros(6, 1);
    dx(1) = x(2);
    dx(2) = (u_ctrl - (g/2)*(m1*sin(2*x(3)) + m2*sin(2*x(5))) - (m1*l1*x(4)^2*sin(x(3))) - (m2*l2*x(6)^2*sin(x(5)))) / (M + m1*sin(x(3))^2 + m2*sin(x(5))^2);
    dx(3) = x(4);
    dx(4) = (dx(2)*cos(x(3)) - g*sin(x(3))) / l1;
    dx(5) = x(6);
    dx(6) = (dx(2)*cos(x(5)) - g*sin(x(5))) / l2;
end

