% Clear the workspace and command window
clc;
clear;

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

% 6 X 6 identity matrix 
C = eye(6); 
D = 0; 
%initial conditions
x0 = [10; 0; 20; 0; 30; 0];
x0_err = [0;0;0;0;0;0];
% check if its controlable
Cont = ctrb(A,B);
disp(Cont)
%Check if its controllable size(A,1) gives the n
if rank(Cont) == size(A,1)
    disp('Since rank = n, system is controllable.');
else
    disp('System is not controllable.');
end
disp("For the given initial values, the state response is:") 
% Simulating for openloop
figure;
sys = ss(A,B,C,D); 
initial(sys,x0);
title('Open-Loop Response');
grid on
%% 

% Design an LQR controller with angle weight and effort
Q = diag([150, 100, 150, 100, 150, 100]); 
R = 0.05;
% LQR gain
[K,pd,poles] = lqr(A, B, Q, R);  
% Defining closed loop ss A,B,C,dD
disp(poles)
A_cl = A - B*K;
B_cl = B;
C_cl = eye(size(A));
D_cl = zeros(size(B));
% Assign state space 
sys_cl = ss(A_cl,B_cl,C_cl,D_cl);
% Simulating for LQR
figure;
initial(sys_cl,x0);
title('Closed-Loop Response');
% Use Lyapunov's indirect method to certify stability
eigenvalues = eig(A_cl);
disp("Using Lyapnunov's indirect methos we can find that")
if all(real(eigenvalues) < 0)
    disp('The closed-loop system is stable.');
else
    disp('The closed-loop system is unstable.');
end
