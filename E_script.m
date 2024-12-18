% Clear the workspace and command window
clc;
clear;

%Defining symbols
syms M m1 m2 l1 l2 g;
% Define the A,B,
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

% C for output vector x(t)
C1 = [1 0 0 0 0 0];

% C for output vector (theta1(t), theta2(t))
C2 = [0 0 1 0 0 0;
      0 0 0 0 1 0];

% C for output vector (x(t), theta2(t))
C3 = [1 0 0 0 0 0;
      0 0 0 0 1 0];

% C for output vector (x(t), theta1(t), theta2(t))
C4 = [1 0 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 0 1 0];
% Checking the Observability Condition

ob1 = [C1' A'*C1' A'*A'*C1' A'*A'*A'*C1' A'*A'*A'*A'*C1' A'*A'*A'*A'*A'*C1'];
ob1_r = rank(ob1);
if (ob1_r==6)
    disp('It is full rank and the system is observable for x(t) as output vector.')
else
    disp('It is not a full rank and the system is not observable for x(t) as output vector.')
end

% Checking the Observability Condition

ob2 = [C2' A'*C2' A'*A'*C2' A'*A'*A'*C2' A'*A'*A'*A'*C2' A'*A'*A'*A'*A'*C2'];

ob2_r = rank(ob2); 
if (ob2_r==6)  % Condition for observability when rank is 6
    disp('It is full rank and the system is observable for theta1(t) and theta2(t) as output vector.')
else
    disp('It is not a full rank and the system is not observable for theta1(t) and theta2(t) as output vector.')
end

% Checking the Observability Condition

ob3 = [C3' A'*C3' A'*A'*C3' A'*A'*A'*C3' A'*A'*A'*A'*C3' A'*A'*A'*A'*A'*C3'];

ob3_r = rank(ob3);
if (ob3_r==6) % Condition for observability when rank is 6
    disp('It is full rank and the system is observable for x(t) and theta2(t) as output vector.')
else
    disp('It is not a full rank and the system is not observable for x(t) and theta2(t) as output vector.')
end

% Checking the Observability Condition

ob4 = [C4' A'*C4' A'*A'*C4' A'*A'*A'*C4' A'*A'*A'*A'*C4' A'*A'*A'*A'*A'*C4'];

ob4_r = rank(ob4);
if (ob4_r==6) % Condition for observability when rank is 6
    disp('It is full rank and the system is observable for x(t), theta1(t) and theta2(t) as output vector.')
else
    disp('It is not a full rank and the system is not observable for x(t), theta1(t) and theta2(t) as output vector.')
end



