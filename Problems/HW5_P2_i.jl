
using ControlSystems;

A=[0 1;1 0];
B=[0;1];
Q=[2 2;2 2];
R=8;

l=lqr(A,B,Q,R)

care(A,B,Q,R)
