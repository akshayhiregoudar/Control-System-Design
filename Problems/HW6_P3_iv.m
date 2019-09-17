sysGs=tf([1],[1 1 0]);
sysDs=tf([6 12 10],[1 0]);

sysGDs=series(sysGs,sysDs);
sysCLs=feedback(sysGDs,1);

step(sysCLs)
