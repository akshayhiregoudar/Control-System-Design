% Problem 3 - Bonus Question

A=[-2 1 0;0 -2 0;0 0 4];
B=[0 0;0 1;1 0];

% First subproblem
K1=place(A,B,[-2,-3,-4])

% Second subproblem
K2=place(A,B,[-2 -2 -20])

% Third subproblem
% 'place' command cannot place poles whose multiplicity is greater than rank(B)=2
% Hence, we will write one of the poles as shown below

K3=place(A,B,[-3 -3 -3e-100])

% Fourth subproblem
K4=place(A,B,[-3 -2+i -2-i])