function x_pred=motion_model(x_1,y_1,theta_1,v,omega,delta_t)
    x = x_1 + -v/omega * sin(theta_1) + v/omega *sin(theta_1+omega*delta_t);
    y = y_1 + v/omega * cos(theta_1) - v/omega * cos(theta_1+omega*delta_t);
    theta = wrapToPi(theta_1+omega*delta_t);
    x_pred = [x,y,theta];
end