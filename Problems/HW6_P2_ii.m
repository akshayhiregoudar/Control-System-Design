n=0:0.1:100;
Xn=10-(16.66.*(0.8).^n)+6.66.*(0.5).^n;
plot(Xn);
sys=tf(Xn);
sysd=c2d(sys,1);
step(sysd)