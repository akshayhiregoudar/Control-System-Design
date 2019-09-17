
# Problem 1 - Subproblem 2

using ControlSystems;

A=[0 0;-1 sqrt(2)]; 
B=[0 -1;0 sqrt(2)]; 
Q=[0 0;0 1]; 
R=[1 0;0 1];

X=care(A,B,Q,R)
