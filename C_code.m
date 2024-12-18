% Clearing the previous outputs

clc
clear
%% 
%the variable in the A and B matrix are
% M is the Mass of cart
% m1 is the pendulum 1 mass 
% m2 is the pendulum 2 mass 
% l1 is the pendulum 1 length  
% l2 is the pendulum 2 length
% g is the gravity 

%Defining symbols
syms M m1 m2 l1 l2 g;
%% 
%defining matrix A and B
A=[0 1 0 0 0 0;
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g/(M*l1)) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -((M+m2)*g/(M*l2)) 0];
disp(A)

B = [0; 
    1/M; 
    0; 
    1/(M*l1); 
    0; 
    1/(M*l2)];
disp(B)

%% 
% Let Cont be the controllability matrix
Cont = [B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B];
disp("The controllability matrix C is ")
disp(Cont)
%% To check determinant
disp("The determinant of controllability matrix is: ")
disp(simplify(det(Cont)));
%% 
%To find the rank of matrix
disp("The rank of matrix is")
rank(Cont)
%Since the rank is full rank, the system is controllable
%% 

%When l1 = l2 the system becomes uncontrollable. To prove it
% Lets see the rank and det of the C matrix
disp("Rank when l1 = l2")
Uncont = subs(Cont, l1, l2);
disp(rank(Uncont))
disp("Thus the rank < 6, which shows that it is uncontrollable")

%Check the determinant of C
disp("Determinant when l1 = l2 is: ")
disp(simplify(det(Uncont)))








