using ControlSystems
using Plots

m=1;
b=1;
k=2;

sys = tf([1,0,1], [m,b,m,b,k]);

impulse(sys)

impulseplot(sys)
