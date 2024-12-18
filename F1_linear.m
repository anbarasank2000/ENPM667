
% Clear the workspace and command window
clc;
clear;

% Define given parameters
M = 1000;  % Mass of cart in Kg
m1 = 100;  % Mass of pendulum 1 in Kg
m2 = 100;  % Mass of pendulum 2 in Kg
l1 = 20;  % Length of pendulum 1 in m
l2 = 10;  % Length of pendulum 2 in m
g = 9.81;  % Acceleration due to gravity in m/s^2

% Define the A,B,C,D matrices
A = [0 1 0 0 0 0;
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -g*(M+m1)/(M*l1) 0 -m2*g/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -m1*g/(M*l2) 0 -g*(M+m2)/(M*l2) 0];

B = [0;
    1/M;
    0;
    1/(M*l1);
    0;
    1/(M*l2)];

Q = diag([150, 100, 150, 100, 150, 100]); 
R = 0.05;
K = lqr(A, B, Q, R);  % LQR gain
D = 0;  % Output matrix

% Define C matrices and observer gains
C_matrices = {[1 0 0 0 0 0], [1 0 0 0 0 0; 0 0 0 0 1 0], [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0]};
poles = [-1;-2;-3;-4;-5;-6];  % Observer poles

% Initial Conditions for Leunberger observer
x0 = [10,0,20,0,30,0,0,0,0,0,0,0];

% Creating observers in a loop
for i = 1:length(C_matrices)
    C = C_matrices{i};
    L = place(A', C', poles)';
    A_obs = [(A-B*K) B*K; zeros(size(A)) (A-L*C)];
    C_obs = [C zeros(size(C))];
    B_all = [B; B];
    
    % Creating the state-space model for each observer
    sys = ss(A_obs, B_all, C_obs, D);
    
    % Simulating initial condition response and step response
    figure;
    initial(sys, x0);
    title(['Initial Condition Response for Observer ', num2str(i)]);
    grid on;
    
    figure;
    step(sys);
    title(['Step Response for Observer ', num2str(i)]);
    grid on;
end


