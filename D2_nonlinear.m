% Clear the workspace and command window
clc;
clear;

% Define initial conditions
x0 = [10; 0; 20; 0; 30; 0];

% Time span for the simulation
tspan = 0:0.01:9000;

% Simulate the system using ode45
[t5, x5] = ode45(@nonlinear,tspan, x0)
% Plot results
plot(t5, x5);
xlabel('Time (s)');
ylabel('State variables');
title('Nonlinear system response');

% Nonlinear dynamics as an anonymous function
function dx = nonlinear(t, x)
   % Define given parameters
M = 1000;  % Mass of cart in Kg
m1 = 100;
m2 = 100;  % Mass of pendulums in Kg
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
    
    Q = diag([1500, 1000, 1000, 800, 950, 700]); 
    R = 0.1;
    % Control input based on LQR controller
    [K, p_dep, pole] = lqr(A,B,Q,R);
    u = -K*x;
    dx=zeros(6,1);
    dx(1) = x(2);
    dx(2)=(u-(g/2)*(m1*sind(2*x(3))+m2*sind(2*x(5)))- (m1*l1*(x(4)^2)*sind(x(3))) ...
           -(m2*l2*(x(6)^2)*sind(x(5))))/(M +m1*((sind(x(3)))^2)+m2*((sind(x(5)))^2));
    dx(3)= x(4); 
    dx(4)= (dx(2)*cosd(x(3))-g*(sind(x(3))))/l1';
    dx(5)= x(6);
    dx(6)= (dx(2)*cosd(x(5))-g*(sind(x(5))))/l2;
end