
using ControlSystems;
using DifferentialEquations;
using PyPlot;
using Polynomials;

A=[0 1;1 0];
B=[0;1];
C=[0 0];
D=0;

sys=ss(A,B,C,D,0);

u(x,t)=[-2.11803 -2.11803 ]*x;
x0=[1;1];
tspan=0:0.01:20;
y, t, x, uout = lsim(sys,u,tspan,x0=x0);

plot(t,x[:,1],label="x1")
plot(t,x[:,2],label="x2")
xlabel("time (t)")
legend()
grid(true)
