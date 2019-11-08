clc;
clear all;
close all;
m = 1; %kg
L = 1;%meter

% Desired altitude and Yaw angle
ZD = 10; %meters
psiD = 10; %rads

% Initializing the system
Z_dd = 1;
Z_d = 0;
Z = 0;

psi_dd = 1;
psi_d = 0;
psi = 0;

K_p1 = 4;
K_d1 = 4;

K_p2 = 5;
K_d2 = 120;

I_z = 1089;

T = 0.1;
k=1;
for t = 0:T:100
    
    Z_dd(k+1,:) = -(1/m) * (K_d1*Z_d(k,:) + K_p1*(Z(k,:) - ZD));
    psi_dd(k+1,:) = -(L/I_z) * (K_d2*psi_d(k,:) + K_p2*(psi(k,:) - psiD));

    Z_d(k+1,:) = Z_d(k,:) + T*Z_dd(k+1,:);
    psi_d(k+1,:) = psi_d(k,:) + T*psi_dd(k+1,:);
    
    Z(k+1,:) = Z(k,:) + T*Z_d(k+1,:);
    psi(k+1,:) = psi(k,:) + T*psi_d(k+1,:);
    
    k = k+1;
end

figure;plot(psi);   
title('yaw')
xlabel('time in x0.1 secs')
ylabel('yaw angle (rads)')

figure;plot(Z);   
title('altitude')
xlabel('time in x0.1 secs')
ylabel('Altitude')
