function [x_hat,Sigma_t]=my_ekf(x_hat_pre,Sigma_t,mu_v,sigma_v,mu_omega,sigma_omega,sigma_x,sigma_y,delta_t,z_t)
    x_t_1 = x_hat_pre(1);
    y_t_1 = x_hat_pre(2);
    theta_t_1 = x_hat_pre(3);
    [x_coor,y_coor,theta] = robot_dynamics(x_t_1,y_t_1,theta_t_1,mu_v,mu_omega,delta_t);
    x_pred = [x_coor,y_coor,theta]';
  
    G_t = [1,0,-mu_v*delta_t*sin(theta_t_1);0,1,delta_t*mu_v*cos(theta_t_1);0,0,1];
    R_t = diag([sigma_v^2*delta_t^2*cos(theta_t_1)*cos(theta_t_1),sigma_v^2*delta_t^2*sin(theta_t_1)*sin(theta_t_1),sigma_omega^2*delta_t^2]);
    Sigma_t_bar = G_t * Sigma_t * G_t' + R_t;
    

    H_t = [1 0 0;0 1 0];
    Q_t = diag([sigma_x^2,sigma_y^2]);

    K_t = Sigma_t_bar*H_t'/(H_t*Sigma_t_bar*H_t'+Q_t);
    x_hat = x_pred + K_t*(z_t-[x_pred(1);x_pred(2)]);
    Sigma_t = (eye(3)-K_t*H_t)*Sigma_t_bar;
end