%Open Loop Root Locus MATLAB Code

%Continuous-time plant
G=tf(37.2021,[1 0.2830 2.7452]);
% Desired closed-loop poles info
zeta=0.7;
wn=1;
% ZOH Discrete equivalent of G(s)
Ts=0.3;
G0=c2d(G,Ts)
%choose bandwidth of system equal to natural frequency
Wbw=bandwidth(G0)
%Discrete-time Controller K(z)=k
%Plot Root Locus
rlocus(G0);
title('Root Locus with K(z)=k')
zgrid(zeta ,wn*Ts);
axis equal

% Discrete-time Controller K(z)=k(z-0.9512)/(z-0.6000)
K1=tf([1 -1.687 0.9186] ,[1 -1.6208 0.6208],Ts);
K=0.02*K1;
Gol=series(K,G0);

%%
%Plot Root Locus
rlocus(Gol);
title(' Root Locus with K(z)')
zgrid(zeta , wn*Ts);
axis equal

%%
Gol=series(K,G0);
Gcl=feedback(Gol,1);
tfinal=20;
[y,t]=step(Gcl,tfinal);
plot(t,y,'*')
grid
xlabel('time (s)')
title('Step responseof close loop')
stepinfo(Gcl)


%Output in response to step disturbance
Gdy=feedback(G0,K);
tfinal=35;
[yd,t]=step(Gdy,tfinal);
subplot(223)
plot(t,yd,'*')
grid
xlabel('time (s)')
title('Response (y(n*Ts)) to step disturbance')
resp_dist=stepinfo(Gdy)
Gs = feedback(K*G0, 1);

% Gru=feedback(K,G0);
% tfinal=20;
% opt = stepDataOptions;
% opt.StepAmplitude = 5.3;
% % [u,t]=step(Gru,tfinal,opt)
% step(Gru,tfinal,opt)
% % plot(t,u,'*')
% grid
% xlabel('time (s)')
% title('Control Signal u(n*Ts)')
% control_signal=stepinfo(Gru)