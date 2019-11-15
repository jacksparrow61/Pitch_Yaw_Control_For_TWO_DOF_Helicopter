%% Discrete Design of Yaw Controller
%Continuous-time plant
G=tf(7.461,[1 0.2701 0]);
% Desired closed-loop poles info
zeta=0.7;
wn=1;
% ZOH Discrete equivalent of G(s)
Ts=0.3;
G0=c2d(G,Ts)
pole(G0)
zero(G0)

%% Discrete-time Controller K(z)=k
%Plot Root Locus
rlocus(G0);
title('Root Locus with K(z)=k')
zgrid(zeta ,wn*Ts);
axis equal
%% Discrete-time Controller
K1=tf([1 -0.89] ,[1 -0.6],Ts)
K2=tf([1 -0.991],[1 -1],Ts)
K=0.113*K1*K2;
Gol=series(K,G0);
%Plot Root Locus
rlocus(Gol);
title(' Root Locus with K(z)')
zgrid(zeta , wn*Ts); axis equal

Gcl=feedback(Gol,1);
tfinal=40;
[y,t]=step(Gcl,tfinal);
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step response of close loop compensated system')
stepinfo(Gcl)

%% Disturbance
Gdy=feedback(G0,K)
tfinal=300;
[yd,t]=step(Gdy,tfinal);
plot(t,yd,'*')
grid
xlabel('time (s)')
title('Response (y(n*Ts)) to step disturbance')
resp_dist=stepinfo(Gdy)