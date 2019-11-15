%% Discrete Design of Pitch Controller 

%Plant
clear
G=tf(37.2021,[1 0.2830 2.7452])

zeta=0.7;
wn=1;

Ts=0.3; % Because t_r = 3 sec therefore  Ts < t_r/6

G0=c2d(G,Ts)
% z=tf('z')
% Ka= .0693*((z-.8433)/(z-0.8827))
% rlocus(Ka*G0)
pole(G0)
zero(G0)
rlocus(G0);
zgrid(zeta,wn*Ts);

%% ContolleR
K1=tf([1 -1.687 .9186],[1 -1.6208 .6208],Ts)
K=0.0227*K1
G1=series(G0,K)
rlocus(G1)

zgrid(zeta,wn*Ts);
Gcl = feedback(G1,1)

%Step response
step(Gcl,200)
stepinfo(Gcl)

%% Disturbance

Gdy=feedback(G0,K)
tfinal=400;
[yd,t]=step(Gdy,tfinal);

plot(t,yd,'*')
grid
xlabel('time (s)')
title('Response (y(n*Ts)) to step disturbance')
resp_dist=stepinfo(Gdy)
