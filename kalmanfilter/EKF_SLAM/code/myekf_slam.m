function [mu_new,Sigma_new] = myekf_slam(state_pre,Sigma_t,v_real,sigma_v,omega_real,sigma_omega,range_real,bear_real,delta_r,delta_phi,delta_t)
   % x_t :ture state 3x1
   % state_pre: last state
    theta = state_pre(3);



    % the 2 landmark and 2 measurement variable, so it is 2N=4
    Fx = [eye(3),zeros(3,4)]; %3x7
    mu_bar = state_pre+Fx'*[-sin(theta)*v_real/omega_real+v_real/omega_real*sin(theta+omega_real*delta_t);
                    v_real/omega_real*cos(theta)-v_real/omega_real*cos(theta+omega_real*delta_t);
                    omega_real*delta_t];
    x_pred = mu_bar(1);
    y_pred = mu_bar(2);
    theta_pred = mu_bar(3);

    dim_of_state = size(state_pre,1);
    temp_theta = [-v_real/omega_real*cos(theta)+v_real/omega_real*cos(theta+omega_real*delta_t);
                    -v_real/omega_real*sin(theta)+v_real/omega_real*sin(theta+omega_real*delta_t);
                    0];
    Jocabian_g = [zeros(3,2),temp_theta];
    G_t = eye(dim_of_state) + Fx'*Jocabian_g*Fx;
    V_t = [(-sin(theta)+sin(theta+omega_real*delta_t))/omega_real, 1/omega_real^2*(v_real*(sin(theta)-sin(theta+omega_real*delta_t)))+1/omega_real*(v_real*cos(theta+omega_real*delta_t)*delta_t);
           (-cos(theta)+cos(theta+omega_real*delta_t))/omega_real, -1/omega_real^2*(v_real*(cos(theta)-cos(theta+omega_real*delta_t)))+1/omega_real*(v_real*sin(theta+omega_real*delta_t)*delta_t); 
            0, delta_t];
    M_t= diag([sigma_v^2,sigma_omega^2]);
    R_t = V_t*M_t*V_t'; % 3x2 2x2 2x3
    Sigma_bar = G_t*Sigma_t*G_t' + Fx'*R_t*Fx;

    Q_t = [delta_r^2,0;
            0,delta_phi^2];
    % number of landmark is 2
    
    for i=1:2
        landmark_idx_x = 3 + 2*i-1; % 4,6
        landmark_idx_y = landmark_idx_x + 1; % 5 7
        if mu_bar(landmark_idx_x) == 0 && mu_bar(landmark_idx_y) == 0
            mu_bar(landmark_idx_x) = x_pred + range_real(i) * cos(theta_pred + bear_real(i));
            mu_bar(landmark_idx_y) = y_pred + range_real(i) * sin(theta_pred + bear_real(i));
        end

        z_measurement = [range_real(i);bear_real(i)];
        
        
        %12-14
        delta_landx_x_t =  mu_bar(landmark_idx_x)-x_pred;
        delta_landy_y_t =  mu_bar(landmark_idx_y)-y_pred;
        q = delta_landx_x_t^2+delta_landy_y_t^2;
        z_hat = [sqrt(q);wrapToPi(atan2(delta_landy_y_t,delta_landx_x_t)-theta_pred)];
        if i==1
            %(5,7)
            F_xj = [eye(3),zeros(3,4);
                zeros(2,3),eye(2),zeros(2,2)]; 
        end
        
        if i==2
            F_xj = [eye(3),zeros(3,4);
                zeros(2,3),zeros(2),eye(2,2)]; 
        end
%         F_xj = [eye(3),zeros(3,4);
%                 zeros(2,3),zeros(2,2*i-2),eye(2),zeros(2*2-2*i)];
        % then we need to calculate the H_t,K_t
        % (2x5)
        H_t = (1/q) *[-sqrt(q)*delta_landx_x_t, -sqrt(q)*delta_landy_y_t,0,sqrt(q)*delta_landx_x_t,sqrt(q)*delta_landy_y_t;
                    delta_landy_y_t, -delta_landx_x_t,-q,-delta_landy_y_t,delta_landx_x_t];
        H_t = H_t*F_xj; %(2x7)

        % 15-17
        %   (7x7)*(7x2) *(2x7x7x7x7x2 + 2x2) = 7x2 * 2x2 = 7x2
        K_t = Sigma_bar*H_t'/(H_t*Sigma_bar*H_t'+Q_t);
        tempZ = [z_measurement(1)-z_hat(1);
            wrapToPi(z_measurement(2)-z_hat(2))];
        mu_bar = mu_bar + K_t*(tempZ); %7x1 + 7x2 * 2x1
        mu_bar(3) = wrapToPi(mu_bar(3));
        Sigma_bar = (eye(dim_of_state)-K_t*H_t)*Sigma_bar;
    end
    mu_new = mu_bar;
    Sigma_new = Sigma_bar;
end