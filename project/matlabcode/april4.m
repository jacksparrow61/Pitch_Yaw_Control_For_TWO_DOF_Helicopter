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

%%
% Design a lead controller to stabilize the system.
% z= tf('z');
K1=tf([1 -1.687 .9186],[1 -1.6208 .6208],Ts)
G1=series(G0,K1)
rlocus(G1)
zgrid(zeta,wn*Ts);

G2 = .02*G1;
z= 0.7920+ 0.1723i
angle(-.208+.1723i)
% abs(z-0.8827)
% (.1947*.1809)/(.1798*2.8266)
% atan(.1723/(.9719+.7920))
% angle(z-.8433+.4555i)
% abs(pi)
% (.7920*(tan(0.0974+1.5708-1.6523+1.75-3.1416))-0.1723)/5.0605
% % 
sys= feedback(G2,1)
step(sys)
stepinfo(sys)




%% 
%Design a Lag because from the step response its clear that huge steady
%state error exists.
K2= tf([1 -.95],[1 -.9998],Ts)
G3= series(K2,G2)

rlocus(G3)
zgrid(zeta,wn*Ts);

sys= feedback(1.08*G3,1)
step(sys)
stepinfo(sys)

%%
%Disturbance 
K1=tf([1 -1.687 .9186],[1 -1.6208 .6208],Ts)
G2=0.02*K1
Gdy=feedback(G0,G2);
tfinal=40;
[yd,t]=step(Gdy,tfinal);

plot(t,yd,'*')
grid
xlabel('time (s)')
title('Response (y(n*Ts)) to step disturbance')
resp_dist=stepinfo(Gdy)

%% Project-2
%Open Loop Root Locus MATLAB Code

%Continuous-time plant
G=tf(7.461,[1 0.2701 0]);
% Desired closed-loop poles info
zeta=0.7;
wn=1;
% ZOH Discrete equivalent of G(s)
Ts=0.3;
G0=c2d(G,Ts)
pole(G0)

%Discrete-time Controller K(z)=k
%Plot Root Locus
rlocus(G0);
title('Root Locus with K(z)=k')
zgrid(zeta ,wn*Ts);
axis equal

%%
% Discrete-time Controller K(z)=k(z-0.9512)/(z-0.6000)
K1=tf([1 -0.89] ,[1 -0.6],Ts);
K2=tf([1 -0.991],[1 -0.999],Ts);
K=0.113*K1*K2;
Gol=series(K,G0);
%Plot Root Locus
rlocus(Gol);
title(' Root Locus with K(z)')
zgrid(zeta , wn*Ts); axis equal


%%
Gol=series(K,G0);
Gcl=feedback(Gol,1);
tfinal=40;
[y,t]=step(Gcl,tfinal);
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step responseof close loop')
stepinfo(Gcl)

%%
%Output in response to step disturbance
Gdy=feedback(G0,K);
tfinal=98;
[yd,t]=step(Gdy,tfinal);
subplot(223)
plot(t,yd,'*')
grid
xlabel('time (s)')
title('Response (y(n*Ts)) to step disturbance')
resp_dist=stepinfo(Gdy)
Gs = feedback(K*G0, 1);

% Gru=feedback(K,G0);
% tfinal=32;
% opt = stepDataOptions;
% opt.StepAmplitude = 5.3;
% % [u,t]=step(Gru,tfinal,opt)
% step(Gru,tfinal,opt)
% % plot(t,u,'*')
% grid
% xlabel('time (s)')
% title('Control Signal u(n*Ts)')
% control_signal=stepinfo(Gru)
