% phi represents Euler angles about x body axis
% theta represents Euler angles about y body axis
% psi represents Euler angles about z body axis
% L is distance from pivot to motor
% I is the body inertial matrix
% I_x, I_y, I_z are the inertia in x, y and z respectively
% u1 is vertical force input
% u2 is pitch moment input
% u3 is roll moment input
% u4 is yaw moment input

%  system
% X and Y are horizontal plane and Z is vertical plane
% 
clear all;
close all;

L =1;           % 1 meter
I_x = 1210 ; 
I_y = 1226
I_z = 1089

phi_dd = 0.5;   %rad
phi_d = 1;      %rad
phi = 0;        %rad

theta_dd = 0.5; %rad
theta_d = 1;    %rad
theta = 0;      %rad

psi_dd = 0.5;   %rad
psi_d = 1.1;    %rad
psi = 0;        %rad

% rads/sec
omega1 = 100;    %CW
omega2 = 100;   %CCW
omega3 = 100;   %CW
omega4 = 100;   %CCW

K_t = 0.5;  % Thrust constant
K_d = 0.7;  % Drag constant

% Inputs
u1 = K_t*(omega1^2 + omega2^2 + omega3^2 + omega4^2);
u2 = K_t*(omega1^2 - omega3^2);
u3 = K_t*(omega2^2 - omega4^2);
u4 = K_d*(omega2^2 + omega4^2 - omega1^2 - omega3^2)

% Sampling time
T = 0.01;
k = 1; 
for t = 0:T:10
    % System Equations
    phi_dd(k+1,:) = (I_y - I_z )/I_x*theta_d(k,:)*psi_d(k,:) + L/I_x*u3;
    theta_dd(k+1,:) = (I_z - I_x)/I_y*phi_d(k,:)*psi_d(k,:) + L/I_y*u2; 
    psi_dd(k+1,:) =  L/I_z*u4;

    phi_d(k+1,:) = phi_d(k,:) + T*phi_dd(k+1,:);
    theta_d(k+1,:) = theta_d(k,:) + T*theta_dd(k+1,:);
    psi_d(k+1,:) = psi_d(k,:) + T*psi_dd(k+1,:);
    
    phi(k+1,:) = phi(k,:) + T*phi_d(k+1,:);
    theta(k+1,:) = theta(k,:) + T*theta_d(k+1,:);
    psi(k+1,:) = psi(k,:) + T*psi_d(k+1,:);
    
    k = k+1;
end

% Plots
figure;plot(theta_d);
title('pitch')
xlabel('time')
ylabel('pitch angle (rads)')
figure;plot(phi_d);
title('roll')
xlabel('time')
ylabel('roll angle (rads)')

figure;plot(psi_d);   
title('yaw')
xlabel('time')
ylabel('yaw angle (rads)')
    