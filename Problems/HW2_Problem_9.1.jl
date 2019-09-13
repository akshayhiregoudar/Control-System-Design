A=[0 2;0 0];
t=(-5,0.5,5);

X1=repmat([t;]',length(t));
X2=repmat([t;],1,length(t));

#Let exp(A*t)=E
E=[1 2.*t;0 1];

using PyPlot

quiver(X1,X2,E,1)
plot(X[:,1],X[:,2],label="Problem_9_1",colour="r")
