A=[-2 1 0;0 -2 0;0 0 4];
B=[0;1;1];
C=[1 0 0;0 0 1];
D=0;

% I have considered the first subproblem from question 3.

% Design an observer
sys=ss(A,B,C,D);
eig(sys);
rank(obsv(A,C))
syms L L_T
L=place(A',C',[-2 -3 -4])';

% Design the controller to combine it with the observer
rank(ctrb(A,B))
K=place(A,B,[-2 -3 -4])

% Initial conditions
x=[1;0;0];
xc=[1;0;0];

xh=zeros(3,1);
step_in=sin(2*pi/100);

ITER=500;
it_ind=[1:ITER];
step_in=ones(1,ITER);

for cnt1 = 1:ITER
    uh(cnt1) = -K*xh(:,cnt1);
    x(:,cnt1+1) = A*x(:,cnt1) + B*uh(cnt1) + step_in(cnt1);
    y(cnt1) = C*x(:,cnt1);
    xh(:,cnt1+1) = (A-L*C-B*K)*xh(:,cnt1) + L*y(cnt1) + step_in(cnt1);
    yh(cnt1) = C*xh(:,cnt1);
    u(cnt1) = -K*xc(:,cnt1);
    xc(:,cnt1+1) = A*xc(:,cnt1) + B*u(cnt1) + step_in(cnt1);
end

% Plots
subplot(321)
plot(x(1,:))
grid on
axis([1 ITER -100 100])

