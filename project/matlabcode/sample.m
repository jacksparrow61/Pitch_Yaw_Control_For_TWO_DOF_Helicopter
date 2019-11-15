% CONTINUOUS TIME PLANT
G= tf([37.2021],[1 .2830 2.7452])

%Discrete Time ZOH controller
T = 0.1  % Since ws>30 wbw
Gd = c2d(G,T)
z= zero(Gd)
p= pole(Gd)

%Desired closed loop poles
zeta = 0.7
w_n = 2.54 % Since T_r<=18 sec and T_r<=3 sec

% Discrete-time Controller K(z)=k
% Root Locus
rlocus(Gd);
title('Root Locus with K(z)=k')
zgrid(zeta , w_n*T);
axis equal

%% 
angle(.8133+0.15096i)
%%
%  Discrete-time Controller K(z)=k(z-.65)/(z-0.9)(z-1)
K1=tf([1 -0.9],[1 -1],T);
K2=tf([1 -.92],[1 -0.01],T);
K4= tf(1.9*conv([1 -.9],[1 -1]),conv([1 -0.92],[1 -0.01]),T)
K3=1.9*K1*K2
K5=series(K1,K2);
K6=1.9*K5
%%
% Open-loop system
Gol=series(K3,Gd);
% Plot Root Locus
rlocus(Gol);
title('Root Locus with K(z)=k(z-0.657)/(z-0.9)(z-1)')
zgrid(zeta , w_n*T);
axis equal

Gcl=feedback(Gol,1)
step(Gcl)
stepinfo(Gcl)
%%
U =feedback(K3,Gd)
tfinal=10;

opt=stepDataOptions;
opt.StepAmplitude=8;
step(U,tfinal,opt)
