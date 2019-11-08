% Position control 

r_1 = -1*k_p1*z_d
u_1 = (r_1 + m*g)/(cos(theta)*cos(phi));

%  Error equations for speeds in x and y directions
e_x = x_d_des - X_d;
e_y = e_d_des - Y_d;

%  Desired rolla nd pitch angles in terms of actual and desired speeds

phi_des = asin(u_e_x*sin(psi) - u_e_y*cos(psi));
theta_des = asin((u_e_x - sin(phi)*sin(psi))/(cos(phi)*cos(psi)));

u_e_x = K_1*e_x*m/u_1;
u_e_y = K_2*e_y*m/u_1;

