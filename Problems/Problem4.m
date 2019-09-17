% Problem 4
% Solved by choosing the first subproblem from Problem 3

A=[-2 1 0;0 -2 0;0 0 4];
B=[0;1;1];
C=[1 0 0;0 0 1];
D=[0];
poles=eig(A)
x0=[0.1 0 0.1];
t=0:0.01:1;
u=zeros(size(t));
sys=ss(A,B,C,D);

% Controller Design
% Verify that the system is controllable
R_c=rank(ctrb(A,B))
K=place(A,B,[-2 -3 -4])
sys_cont=ss(A-B*K,B,C,0);
figure(1)
lsim(sys_cont,u,t,x0);
title('Controller Simulation')
xlabel('Time(s)')

% Observer Design
% Verify that the system is observable
R_o=rank(obsv(A,C))
L=place(A',C',[-2 -3 -4])'
At=[A-B*K B*K;zeros(size(A)) A-L*C];
Bt=[B;zeros(size(B))];
Ct=[C zeros(size(C))];
sys_obs=ss(At,Bt,Ct,0);
figure(2)
lsim(sys_obs,zeros(size(t)),t,[x0 x0]);
title('Observer Simulation')
xlabel('Time(s)')

% Combined Simualtion
t = 0:0.01:1;
[y,t,x] = lsim(sys_obs,zeros(size(t)),t,[x0 x0]);

n = 3;
e = x(:,n+1:end);
x = x(:,1:n);
x_est = x - e;

h = x(:,1); h_dot = x(:,2); i = x(:,3);
h_est = x_est(:,1); h_dot_est = x_est(:,2); i_est = x_est(:,3);

figure(3)
plot(t,h,'-r',t,h_est,':r',t,h_dot,'-b',t,h_dot_est,':b',t,i,'-g',t,i_est,':g')
legend('h','h_ est','hdot','hdot_ est','i','i_ est')
xlabel('Time (sec)')
title('Combined Simulation')