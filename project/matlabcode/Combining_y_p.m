%% Discrete–Time Controller Design For Complete System

% Complete Plant

zeta=0.7;
wn=1;

Ts=0.3; % Because t_r = 3 sec therefore  Ts < t_r/6

A= [0 1 0 0; -2.7451 -.2829 0 0; 0 0 0 1; 0 0 0 -.2701];
B= [0 0; 37.2021 3.5306; 0 0; 2.3892 7.461];
C= [1 0 0 0;0 0 1 0];
D= [0];
G= ss(A,B,C,D)

G0=c2d(G,Ts)

%% Contollers
Ts=0.3
K_1=tf([1 -1.687 .9186],[1 -1.6208 .6208],Ts);
K_p=0.0227*K_1;
Kp_s= ss(K_p)

K2=tf([1 -0.89] ,[1 -0.6],Ts);
K3=tf([1 -0.991],[1 -1],Ts);
K_y=0.13*K3*K2;
Ky_s= ss(K_y)

%% Combining
clc
Kp_s
Ky_s
K= append(Kp_s,Ky_s)
G0
Gol = series(K,G0)

Gcl= feedback(Gol,eye(2))
step(Gcl)
s=stepinfo(Gcl)
s(1,1)
s(1,2)
s(2,1)
s(2,2)

[y,t,x]=step(Gcl);
plot(t,y(:,:,1),'*')
grid
xlabel('time (s)')
title('Step Response of pitch reference to theta and psi')
hold on
plot(t,y(:,:,2),'*')
grid
xlabel('time (s)')
title('Step Response of pitch and yaw reference to theta and psi')

