function [x,y,theta] = robot_dynamics(x_1,y_1,theta_1,v,omega,delta_t)
    x = x_1 + cos(theta_1)*v*delta_t;
    y = y_1 + sin(theta_1)*v*delta_t;
    theta = theta_1 + omega * delta_t;
    theta = mod(theta,2*pi);
end