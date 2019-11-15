% CONTINUOUS TIME PLANT
G= tf([37.2021],[1 .2830 2.7452])

%Discrete Time ZOH controller
T = 0.359  % Since t_r = 3 sec therefore T=t_r/10 i.e w_s>20*w_n
Gd = c2d(G,T)
z= zero(Gd)
p= pole(Gd)

%Desired closed loop poles
zeta = 0.5 
w_n = 0.6 % Since T_r<=18 sec and T_r<=3 sec

% Discrete-time Controller K(z)=k
% Root Locus
rlocus(Gd);
title('Root Locus with K(z)=k')
zgrid(zeta , w_n*T);
axis equal

z1=.1913+.0166i;
a = abs(z1);
% 
% % Discrete-time Controller K(z)=k(z-.65)/(z-0.9)(z-1)
K1=tf([1 -.672],[1 -1],T)
K2=tf([1],[1 -0.8],T)
K3= K1*K2%tf(conv([1 -0.8],[1 -1]),conv([1],[1 -0.9]),T)
% Open-loop system
Gol=series(K3,Gd);
% Plot Root Locus
rlocus(Gol);
title('Root Locus with K(z)=k(z-0.657)/(z-0.9)(z-1)')
zgrid(zeta , w_n*T);
axis equal

% % From root locus selected k=0.0028 as gain
% Step response of closed loop system
Kf= 0.0038*K3
Kc= tf(0.00279*conv([1],[1 -0.9]),conv([1 -0.65],[1 -1]),T)
GolF=series(Kf,Gd)
Gcl=feedback(GolF,1);
abs(pole(Gcl))  % poles inside unit circle
tfinal=50;
[y,t]=step(Gcl,tfinal);
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step response, K(z)=0.00348(z-.7)/(z-0.9)(z-1)')
stepinfo(Gcl)
% % 
% % %Control signal for step input
% % Gru=feedback(Kf,Gd)
% % tfinal=20;
% % [u,t]=step(Gru,tfinal);
% % subplot(222)
% % plot(t,u,'*')
% % grid
% % xlabel('time (s)')
% % title('Control Signal u(n*Ts)')
% % control_signal=stepinfo(Gru)
% % 
% % % Output in response to step disturbance
% % Gdy=feedback(Gd,Kf);
% % tfinal=20;
% % [yd,t]=step(Gdy,tfinal);
% % subplot(223)
% % plot(t,yd,'*')
% % grid
% % xlabel('time (s)')
% % title('Response (y(n*Ts)) to step disturbance')
% % resp_dist=stepinfo(Gdy)