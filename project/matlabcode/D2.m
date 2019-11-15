clc;
clf;

Ts = 0.3;

n = 37.2021;
d = [1 0.283 2.7452];
G = tf(n,d);

Gz = c2d(G,Ts)
pole(Gz)
zero(Gz)
rlocus(Gz)
