clc;
clear all;
close all;
% amount of time in 0.1*secs
N = 10;

% Theta is Pitch angle
% Phi is roll angle
% psi is yaw angle
% Initializing values
theta = 0;
theta_d = 0.1;
theta_dd = 1;

phi = 0;
phi_d = 0.1;
phi_dd = 1;

psi = 0;
psi_d = 0.1;
psi_dd = 1;

X_dd = 2;
X_d = 1;
X = 1;

Y_dd = 2;
Y_d = 1;
Y = 1;

Z = 1; % in m
Z_d = 1; % in m/s
Z_dd = 1;

e_x = 2;
e_y = 3;

e = 2;
e_d = 2;
e_I = 1;

u_1 = 20;
u_2 = 2;
u_3 = 1;
u_4 = 2;

% desired values
X_des_d = 0.1; % in m/s
Y_des_d = 0.1; % in m/s
Z_des = 3; % in m
theta_des = 10; % in rads
theta_des_d = 0.2; % in rads/sec

% I_x, I_y, I_z are Inertia in respective direction (in our system I_x =
% I_y)
I_x = 1198;
I_y = 1198;
I_z = 1089;

% L is the length of the arm from center to the motor in meters
L = 0.1;
% m is mass of the total system in kg
m = 1;
% g is the gravity of earth
g = 9.8; % in m/sec^2

% Constants
K_1 = 3;K_2 = 2;
kp1 = 20; kd1 = 3;
kp2 = 20; kd2 = 3;
k1 = 1;k2 = 1;k3 = 1;k4 = 1;k5 = 1;k6 = 1;
c1 = 4;c2 = 4;c3 = 4;c4 = 4;c5 = 4;c6 = 4;

k = 1; %Increment constant
% System 
dT = 0.1;
for T = 0:dT:N
    
    % Defining u_1 (Vertical force input)
    
        % Z is the altitude of the quadcopter
        % Z_des is desired altitude
        % psi_des is desired yaw angle
        % kd1, kp1, kd2 and kp2 are diff and prop gain consts
        
    r_1(k,:) = -kd1*Z_d(k,:) - kp1*(Z(k,:) - Z_des);
    u_1(k,:) = (r_1(k,:) + m*g)/(cos(theta(k,:))* cos(phi(k,:)));
    
    % Defining u_2 (pitch moment input)
        % k1,k2 are positive tuning parameter
        % c1, c2 are positive constants to increase speed of the pitch and
        % roll tracking loop convergence 
        
        A_u2 = c1*c2 + (c2*k2)/k1;
        B_u2 = (c1*c2*k2)/k1;
        
        K_P_u2 = (k1^2) + (c1*k2)/k1 + A_u2;
        K_I_u2 = k1*k2 + B_u2;
        K_D_u2 = c1 + k2/k1 + c2;
                     
    u_2(k,:) = (I_y/L)*(-e(k,:)*K_P_u2 -K_D_u2*e_d(k,:) - ((I_z - I_x)/I_y)*phi_d(k,:)*psi_d(k,:) - K_I_u2*e_I(k,:) + phi_dd(k,:));
    
    
    % Defining u_3 (roll moment input)
        % k3,k4 are positive tuning parameter
        % c3, c4 are positive constants to increase speed of the pitch and
        % roll tracking loop convergence 
                
        A_u3 = c3*c4 + (c4*k4)/k3;
        B_u3 = (c3*c4*k4)/k3;
        
        K_P_u3 = (k3^2) + (c3*k4)/k3 + A_u3;
        K_I_u3 = k3*k4 + B_u3;
        K_D_u3 = c3 + k4/k3 + c4;
                     
    u_3(k,:) = (I_x/L)*(-e(k,:)*K_P_u3 -K_D_u3*e_d(k,:) - ((I_y - I_z)/I_x)*theta_d(k,:)*psi_d(k,:) - K_I_u3*e_I(k,:) + theta_dd(k,:));
    
    % Defining u_4 (pitch moment input)
        % k5,k6 are positive tuning parameter
        % c5, c6 are positive constants to increase speed of the pitch and
        % roll tracking loop convergence 
                
        A_u4 = c5*c6 + (c6*k6)/k5;
        B_u4 = (c5*c6*k6)/k5;
        
        K_P_u4 = (k5^2) + (c5*k6)/k5 + A_u4;
        K_I_u4 = k5*k6 + B_u4;
        K_D_u4 = c5 + k6/k5 + c6;
                     
    u_4(k,:) = (I_z/L)*(-e(k,:)*K_P_u4 -K_D_u4*e_d(k,:) - K_I_u4*e_I(k,:) + psi_dd(k,:));
    
    X_dd(k+1,:) = (cos(phi(k,:))*sin(theta(k,:))*cos(psi(k,:)) + sin(phi(k,:))*sin(psi(k,:)))*u_1(k,:)/m;
    Y_dd(k+1,:) = (cos(phi(k,:))*sin(theta(k,:))*sin(psi(k,:)) + sin(phi(k,:))*cos(psi(k,:)))*u_1(k,:)/m;
    Z_dd(k+1,:) = -(1/m)*(r_1(k,:));
    
    X_d(k+1,:) = X_d(k,:) + dT*X_dd(k+1,:);
    Y_d(k+1,:) = Y_d(k,:) + dT*Y_dd(k+1,:);
    Z_d(k+1,:) = Z_d(k,:) + dT*Z_dd(k+1,:);
    
    X(k+1,:) = X(k,:) + dT*X_d(k+1,:);
    Y(k+1,:) = Y(k,:) + dT*Y_d(k+1,:);
    Z(k+1,:) = Z(k,:) + dT*Z_d(k+1,:);
    
    % Defining theta desired
        % Defining u_e_x, u_e_y as pitch is dependent on x and y directions
            % Defining error equations of x and y positions
            e_x(k,:) = X_des_d - X_d(k,:);
            e_y(k,:) = Y_des_d - Y_d(k,:);
            
        u_e_x(k,:) = K_1*e_x(k,:)*m/u_1(k,:);
        u_e_y(k,:) = K_2*e_y(k,:)*m/u_1(k,:);
                       
    theta_des(k,:) = asin((u_e_x(k,:) - sin(phi(k,:))*sin(psi(k,:)))/(cos(phi(k,:))*cos(psi(k,:))));
    
    % Defining phi_dd, theta_dd, psi_dd as system equations
    
    phi_dd(k+1,:) = (I_y - I_z)/I_x*theta_d(k,:)*psi_d(k,:) + L/I_x*u_3(k,:);
    theta_dd(k+1,:) = (I_z - I_x)/I_y*phi_d(k,:)*psi_d(k,:) + L/I_y*u_2(k,:); 
    psi_dd(k+1,:) =  L/I_z*u_4(k,:);

    phi_d(k+1,:) = phi_d(k,:) + dT*phi_dd(k+1,:);
    theta_d(k+1,:) = theta_d(k,:) + dT*theta_dd(k+1,:);
    psi_d(k+1,:) = psi_d(k,:) + dT*psi_dd(k+1,:);
    
    phi(k+1,:) = phi(k,:) + dT*phi_d(k+1,:);
    theta(k+1,:) = theta(k,:) + dT*theta_d(k+1,:);
    psi(k+1,:) = psi(k,:) + dT*psi_d(k+1,:);
    
    % defining error equation
    e(k+1,:) = theta(k+1,:) - theta_des(k,:);
    
    % defining e_d
    
    e_d(k+1,:) =  theta_d(k+1,:) - theta_des_d;
    
    e_I(k+1,:) = e_I(k,:) + dT*e(k,:);
    
    k = k+1;
end
 
% Plots

k = 1:N/dT + 2;
figure(1);
plot(k,theta);
title('theta');
xlabel('time in secs');
ylabel('rads');

%plot(phi)
% plot(psi)
% 
% plot(X)
% plot(Y)
% plot(Z)

% k = 1:N/dT + 1;
% figure(2);
% plot(k-1,u_1);
% title('u_1');
