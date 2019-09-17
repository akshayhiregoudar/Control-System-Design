% The order of the system is 3
% Order of output matrix(y) is 2
% Therefore, we have to estimate n-r= 1 state which is x2 in this case

A=[-2 1 0;0 -2 0;0 0 4];
B=[0;1;1];
C=[1 0 0;0 1 0];
D=[0];

y=size(C,1) % Number of outputs
n=size(A,1) % Dimension of system

R = [1 1 1];
W = [C;R];
V = inv(W);

Abar=inv(V)*A*V;
Bbar=inv(V)*B;
Cbar=C*V;

i1=1:y;
i3=y+1:n;

Abar11=Abar(i1,i1)
Abar13=Abar(i1,i3)
Abar21=Abar(i3,i1)
Abar22=Abar(i3,i3)

Bbar1=Bbar(i1,:);
Bbar2=Bbar(i3,:);

% place desired eigs of Abar22-L*Abar12 at -1,-1
% Ackermann's formula

phides=Abar22^2+2*Abar22+eye(2);
Q=obsv(Abar22,Abar13)
q=inv(Q*[1 1]);
L=phides*q

% Form matrices N1, N2, N3
N1 = Abar22-L*Abar13
N2 = N1*L+Abar21-L*Abar11;
N3 = Bbar2-L*Bbar1-N2*D;

V1=V(:,i1);
V2=V(:,i3);

x0=zeros(3,1); % IC of plant
x0hat=[-1;1;-1]; % IC of observer is taken to be consistent with measurements y(0)=u(0)=0
zhat0=W*x0hat;
w0=zhat0(i3);

Ab=[A zeros(3,2);N2*C N1];
Bb=[B;N3+N2*D];
Cb=[eye(3) zeros(3,2);(V1+V2*L)*C V2];
Db=zeros(6,1);

tmax=10;
t=0:.1:tmax;
u=ones(size(t));
[y,t,x]=lsim(ss(Ab,Bb,Cb,Db),u,t,[x0;w0]);
figure(1)
plot(t,y(:,1)-y(:,4),t,y(:,2)-y(:,5),t,y(:,3)-y(:,6),'LineWidth',2)
title('State Estimate Errors (MATLAB CS Toolbox)')
xlabel('Time (s)')
ylabel('1','Interpreter','latex','String','$$\tilde{x}$$','Fontsize',16)


sim('obs_sim',tmax)
figure(2)
plot(tout,xout(:,1:3)-simout1,'LineWidth',2)
title('State Estimate Errors (Simulink)')
xlabel('Time (s)')
ylabel('1','Interpreter','latex','String','$$\tilde{x}$$','Fontsize',16)

figure(3)
plot(tout,xout(:,1:3),'LineWidth',2)
title('System State Trajectories')
xlabel('Time (s)')
ylabel('1','Interpreter','latex','String','$$x$$','Fontsize',16)