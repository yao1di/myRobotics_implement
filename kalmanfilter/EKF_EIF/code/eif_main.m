% Simulation step and time
close all;
clear;
randn('state',0);
delta_t = 0.5;
tf = 30;
t = 0:delta_t:tf;

% initial value
x0 = 5;
y0 = 4;
theta0 = 0+1e-7;


% nose of velocity and angular vecolity
mu_v = 1.5;
sigma_v = sqrt(0.4);
mu_omega=0.5;
sigma_omega = sqrt(0.1);


% measurement noise
mu_deltaX=0;
sigma_deltaX=sqrt(0.2);
mu_deltaY=0;
sigma_deltaY=sqrt(0.3);

x_hat = [x0;y0;theta0];
% R = diag([sigma_v^2*cos(theta0)^2*delta_t^2,sigma_v^2*sin(theta0)^2*delta_t^2,sigma_omega^2*delta^2]);
Sigma_t = diag([sigma_v^2*cos(theta0)^2*delta_t^2,sigma_v^2*sin(theta0)^2*delta_t^2,sigma_omega^2*delta_t^2]);
% Sigma_t = diag([0.01,0.01,1]);
xi_t = pinv(Sigma_t)*x_hat;
Omega_t = pinv(Sigma_t);
% Q = diag([sigma_deltaX^2,sigma_deltaY^2]);

true_path = zeros(3,length(t)+1);
true_path(:,1) = x_hat;
x_t = x0;
y_t = y0;
theta_t = theta0;
path_of_store(:,1)=x_hat;
for i=1:length(t)
    % input for the robot.
    v_real = mu_v+ sigma_v *randn;
    omega_real = mu_omega + sigma_omega*randn;

    [x_t,y_t,theta_t] = robot_dynamics(x_t,y_t,theta_t,v_real,omega_real,delta_t);
    true_path(:,i+1) = [x_t,y_t,theta_t]';

    %measurement data
    z_x = x_t +sigma_deltaX *randn;
    z_y = y_t + sigma_deltaY*randn;
    z_t = [z_x,z_y]';

    [xi_t,Omega_t] = my_eif(xi_t,Omega_t,mu_v,sigma_v,mu_omega,sigma_omega,sigma_deltaX,sigma_deltaY,delta_t,z_t);
    x_hat = pinv(Omega_t)*xi_t;
    path_of_store(:,i+1) = x_hat;
end

figure(1);
plot(true_path(1,:),true_path(2,:),'k','LineWidth',2);
hold on;
plot(path_of_store(1,:),path_of_store(2,:),'r-.','LineWidth',2);
plot(x0,y0,'kp','MarkerSize',12)
grid on;
title('The path of the robot using EIF algorithm.');
legend({'The true path','The EIF path','The start point'},'Location','southeast');
