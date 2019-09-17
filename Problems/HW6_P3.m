% Problem 3

% Subporblem iv

sysGs=tf([1],[1 1 0]);
sysDs=tf([6 12 10],[1 0]);
sysGDs=series(sysGs,sysDs);
sysCLs=feedback(sysGDs,1);
figure(1)
step(sysCLs)
title('Step Response of PID Controller')

%Subproblem v

% For T=1s
sys2=tf([29 -14 5],[1 0 1],1);
sysd1=feedback(sys2,1);
figure(2)
subplot(2,2,1)
step(sysd1)
title('Step Response when T=1s')

% For T=0.1s
sys2=tf([132.5 -239 108.5],[1 0 1],0.1);
sysd2=feedback(sys2,1);
figure(2)
subplot(2,2,2)
step(sysd2)
title('Step Response when T=0.1s')

% For T=0.01s
sys3=tf([1212.05 -2399.9 1188.05],[1 0 1],0.01);
sysd3=feedback(sys3,1);
subplot(2,2,3)
step(sysd3)
title('Step Response when T=0.01s')

% Figure 1 shows individual step responses at various values of T.

% In figure 4 we will plot all of the step responses together.
subplot(2,2,4)
step(sysd1)
hold on
step(sysd2)
hold on
step(sysd3)
hold off
title('Combined Step Response')
legend('T=1s','T=0.1s','T=0.01s')

% Figure 3 shows the comparision of digital controller at various T with a continuous PID controller.
figure(3)
step(sysCLs)
hold on
step(sysd1)
hold on
step(sysd2)
hold on
step(sysd3)
hold off
title('Comparing the Step Responses of subproblems iv and v')
legend('Continuous PID controller','Dig.Cont when T=1s','Dig.Cont when T=0.1s','Dig.Cont when T=0.01s')