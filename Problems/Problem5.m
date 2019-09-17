% Problem 5

A=[-2 1 0;0 -2 0;0 0 4];
B=[0;1;1];
C=[1 0 0;0 0 1];
D=0;

syms t1 t2 t3
T=[t1 t2 t3];
TA=T*A;
F=-3;
FT=F*T;
G=[1 1];
GC=G*C;
X=(TA-FT-GC);
a1=X(1,1)==0;
a2=X(1,2)==0;
a3=X(1,3)==0;
sol=solve([a1 a2 a3],[t1 t2 t3]);
Tvalue = structfun(@double,sol);
P=[C;(Tvalue)'];
H=T*B;

%inverse of P gives reduced order matrix
inv(P)