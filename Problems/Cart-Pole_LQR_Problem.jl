using ControlSystems;
using DifferentialEquations;
using PyPlot;
using Polynomials;

M=2;m=0.1;l=0.5;g=9.81;
A=[0 1 0 0;(m+M)/M/l*g 0 0 0;0 0 0 1;-m*g/M 0 0 0];
B=[0;-1/M/l;0;1/M];
C=[1.0 0 0 0];
D=0.0;
sys=ss(A,B,C,D,0);

eigvals(A)

rank(ctrb(sys))

rank(obsv(sys))

u(x,t)=[0.0 0.0 0.0 0.0]*x;
x0=[0.1;0.0;0.0;0.0];
tspan=0:0.01:10;
y, t, x, uout = lsim(sys,u,tspan,x0=x0);

plot(t,x[:,1],label="theta")
plot(t,x[:,2],label="theta_dot")
plot(t,x[:,3],label="pos")
plot(t,x[:,4],label="vel")
xlabel("time (t)")
legend()
grid(true)

mutable struct Param
    A::Array{Float64,2}
    B::Array{Float64,1}
    C::Array{Float64,2}
    D::Float64
    K::Array{Float64,2}
    L::Array{Float64,2}
end

K=[0.0 0.0 0.0 0.0];
L=[0.0 0.0;0.0 0.0;0.0 0.0;0.0 0.0];
param= Param(A,B,C,D,K,L);

function f!(dx,x,param,t)
    A=param.A;
    B=param.B;
    u=0.0;
    temp=A*x+B*u;
#     make sure that you assign to each element
    dx[1]=temp[1];
    dx[2]=temp[2];
    dx[3]=temp[3];
    dx[4]=temp[4];

end;

prob=ODEProblem(f!,x0,(0.0,10.0),param);
sol=solve(prob,Tsit5(),dtmax=0.01);

n=length(sol.t);
theta=zeros(n); [theta[i]=sol.u[i][1] for i=1:n];
theta_dot=zeros(n); [theta_dot[i]=sol.u[i][2] for i=1:n];
pos=zeros(n); [pos[i]=sol.u[i][3] for i=1:n];
vel=zeros(n); [vel[i]=sol.u[i][4] for i=1:n];

plot(sol.t,theta,label="theta")
plot(sol.t,theta_dot,label="theta_dot")
plot(sol.t,pos,label="pos")
plot(sol.t,vel,label="vel")
xlabel("time (t)")
legend()
grid(true)

v1=nullspace([-2*eye(4)-sys.A sys.B]);
v2=nullspace([-3*eye(4)-sys.A sys.B]);
v3=nullspace([-4*eye(4)-sys.A sys.B]);
v4=nullspace([-5*eye(4)-sys.A sys.B]);

K=[v1[5,1] v2[5,1] v3[5,1] v4[5,1]]*inv([v1[1:4,1] v2[1:4,1] v3[1:4,1] v4[1:4,1]])

sys_new=sys.A-sys.B*K

eigvals(sys_new)

K1=place(sys,[-2,-3,-4,-5])

u(x,t)=-K*x;
y, t, x, uout = lsim(sys,u,tspan,x0=x0);

plot(t,x[:,1],label="theta")
plot(t,x[:,2],label="theta_dot")
plot(t,x[:,3],label="pos")
plot(t,x[:,4],label="vel")
xlabel("time (t)")
legend()
grid(true)

C=[1.0 0 0 0;0.0 0 1 0]
rank(obsv(sys.A,C))

function f!(dx,x,param,t)
    A=param.A;
    B=param.B;
    K=param.K;
    L=param.L;
    
    xtrue=x[1:4];
    xhat=x[5:8];
    u=-K*xhat;
    
    # observer    
    temp_hat=A*xhat+B*u[1]+L*C*(xhat-xtrue);
    
    dx[5]=temp_hat[1];
    dx[6]=temp_hat[2];
    dx[7]=temp_hat[3];
    dx[8]=temp_hat[4];
    
    # true physical system
    temp_true=A*xtrue+B*u[1];  

    dx[1]=temp_true[1];
    dx[2]=temp_true[2];
    dx[3]=temp_true[3];
    dx[4]=temp_true[4];

end;

v1=nullspace([-2*eye(4)-A' C'])
v2=nullspace([-3*eye(4)-A' C'])
v3=nullspace([-4*eye(4)-A' C'])
v4=nullspace([-5*eye(4)-A' C'])

L=[v1[5:6,1] v2[5:6,1] v3[5:6,1] v4[5:6,1]]*inv( [v1[1:4,1] v2[1:4,1] v3[1:4,1] v4[1:4,1]]);
L=L';
eigvals(A-L*C)

# L=place(A',C',[-10,-11,-12,-13])
# L=[22.9338 1.0388;151.3808 11.9770;0.9570 23.0662;10.4925 132.2187];
param.C=C;
param.K=K;
param.L=-L;

x0=[0.1,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
prob=ODEProblem(f!,x0,(0.0,10.0),param);
sol=solve(prob,Tsit5(),dtmax=0.01);

n=length(sol.t);
theta=zeros(n); [theta[i]=sol.u[i][1] for i=1:n];
theta_dot=zeros(n); [theta_dot[i]=sol.u[i][2] for i=1:n];
pos=zeros(n); [pos[i]=sol.u[i][3] for i=1:n];
vel=zeros(n); [vel[i]=sol.u[i][4] for i=1:n];
theta_hat=zeros(n); [theta_hat[i]=sol.u[i][5] for i=1:n];
theta_dot_hat=zeros(n); [theta_dot_hat[i]=sol.u[i][6] for i=1:n];
pos_hat=zeros(n); [pos_hat[i]=sol.u[i][7] for i=1:n];
vel_hat=zeros(n); [vel_hat[i]=sol.u[i][8] for i=1:n];

figure(figsize=(10,10))
subplot(221)
plot(sol.t,theta,label="theta")
plot(sol.t,theta_hat,label="theta_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(222)
plot(sol.t,theta_dot,label="theta_dot")
plot(sol.t,theta_dot_hat,label="theta_dot_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(223)
plot(sol.t,pos,label="pos")
plot(sol.t,pos_hat,label="pos_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(224)
plot(sol.t,vel,label="vel")
plot(sol.t,vel_hat,label="vel_hat")
xlabel("time (t)")
legend()
grid(true)

# place(A',C',[-10,-15,-20,-25])
L=[7.7024 -0.7207;34.5649 -2.5235;-0.7029 6.2976;-2.9390 9.0361];     
param.L=-L;

prob=ODEProblem(f!,x0,(0.0,10.0),param);
sol=solve(prob,Tsit5(),dtmax=0.01);

n=length(sol.t);
theta=zeros(n); [theta[i]=sol.u[i][1] for i=1:n];
theta_dot=zeros(n); [theta_dot[i]=sol.u[i][2] for i=1:n];
pos=zeros(n); [pos[i]=sol.u[i][3] for i=1:n];
vel=zeros(n); [vel[i]=sol.u[i][4] for i=1:n];
theta_hat=zeros(n); [theta_hat[i]=sol.u[i][5] for i=1:n];
theta_dot_hat=zeros(n); [theta_dot_hat[i]=sol.u[i][6] for i=1:n];
pos_hat=zeros(n); [pos_hat[i]=sol.u[i][7] for i=1:n];
vel_hat=zeros(n); [vel_hat[i]=sol.u[i][8] for i=1:n];

figure(figsize=(10,10))
subplot(221)
plot(sol.t,theta,label="theta")
plot(sol.t,theta_hat,label="theta_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(222)
plot(sol.t,theta_dot,label="theta_dot")
plot(sol.t,theta_dot_hat,label="theta_dot_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(223)
plot(sol.t,pos,label="pos")
plot(sol.t,pos_hat,label="pos_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(224)
plot(sol.t,vel,label="vel")
plot(sol.t,vel_hat,label="vel_hat")
xlabel("time (t)")
legend()
grid(true)

K=lqr(sys,diagm([1,1,100,100]),1)

param.K=K;

prob=ODEProblem(f!,x0,(0.0,10.0),param);
sol=solve(prob,Tsit5(),dtmax=0.01);

n=length(sol.t);
theta=zeros(n); [theta[i]=sol.u[i][1] for i=1:n];
theta_dot=zeros(n); [theta_dot[i]=sol.u[i][2] for i=1:n];
pos=zeros(n); [pos[i]=sol.u[i][3] for i=1:n];
vel=zeros(n); [vel[i]=sol.u[i][4] for i=1:n];
theta_hat=zeros(n); [theta_hat[i]=sol.u[i][5] for i=1:n];
theta_dot_hat=zeros(n); [theta_dot_hat[i]=sol.u[i][6] for i=1:n];
pos_hat=zeros(n); [pos_hat[i]=sol.u[i][7] for i=1:n];
vel_hat=zeros(n); [vel_hat[i]=sol.u[i][8] for i=1:n];

figure(figsize=(10,10))
subplot(221)
plot(sol.t,theta,label="theta")
plot(sol.t,theta_hat,label="theta_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(222)
plot(sol.t,theta_dot,label="theta_dot")
plot(sol.t,theta_dot_hat,label="theta_dot_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(223)
plot(sol.t,pos,label="pos")
plot(sol.t,pos_hat,label="pos_hat")
xlabel("time (t)")
legend()
grid(true)
subplot(224)
plot(sol.t,vel,label="vel")
plot(sol.t,vel_hat,label="vel_hat")
xlabel("time (t)")
legend()
grid(true)

