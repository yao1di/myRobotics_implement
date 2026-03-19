function [xi_t,Omega_t]=my_eif(xi_t_1,Omega_t_1,mu_v,sigma_v,mu_omega,sigma_omega,sigma_x,sigma_y,delta_t,z_t)
    mu_t_1 = Omega_t_1\xi_t_1;
    x_t_1 = mu_t_1(1);
    y_t_1 = mu_t_1(2);
    theta_t_1 = mu_t_1(3);
    [xcorr,ycorr,theta] = robot_dynamics(x_t_1,y_t_1,theta_t_1,mu_v,mu_omega,delta_t);
    x_pred = [xcorr,ycorr,theta]';
    
    G_t = [1,0,-mu_v*delta_t*sin(theta_t_1);0,1,delta_t*mu_v*cos(theta_t_1);0,0,1];
    R_t = diag([sigma_v^2*delta_t^2*cos(theta_t_1)*cos(theta_t_1), ...
        sigma_v^2*delta_t^2*sin(theta_t_1)*sin(theta_t_1), ...
        sigma_omega^2*delta_t^2]);
    Omega_t_bar = inv(G_t*pinv(Omega_t_1)*G_t'+R_t);

    
    H_t = [1 0 0;0 1 0];
    Q_t = diag([sigma_x^2,sigma_y^2]);
    xi_t_bar = Omega_t_bar*x_pred;
    mu_t_bar = x_pred;
    Omega_t = H_t'*inv(Q_t)*H_t + Omega_t_bar;
    xi_t = xi_t_bar + H_t'*inv(Q_t)*(z_t-[x_pred(1);x_pred(2)]+H_t*mu_t_bar);
end