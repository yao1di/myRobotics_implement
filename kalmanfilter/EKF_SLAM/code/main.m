clc;
clear;
close all;

%% Initialization the landmarks and robot
x0=6;
y0=3;
theta0=pi/2;


% landmarks locations
landmark1 = [2,6];
landmark2 = [10,7];

% delta_t = 1;
% Tf = 10;

% the translational velocity
mu_v = 2;
sigma_v = sqrt(0.2);

% the rotational velocity
mu_omega = 1;
sigma_omega =sqrt(0.1);

% the robot measure r1,r2,phi1,phi2
% the landmark measurement 
delta_r = sqrt(0.1);
delta_phi = sqrt(0.1);

% state vectors
% the robot and the landmarks
state0 = [x0,y0,theta0,0,0,0,0]';
%  initializition of the state covariance matrix

dimOfstate=length(state0);

landmarks = [landmark1;landmark2];
% %error
%Sigma0 = diag([sigma_v^2*cos(theta0)^2*delta_t^2,sigma_v^2*sin(theta0)^2*delta_t^2,sigma_omega^2*delta_t^2]);
mu_x0 = zeros(dimOfstate,1);
Sigma0 = diag([sigma_v^2,sigma_v^2,sigma_omega^2]);
Sigma_newSystem = diag([sigma_v^2,sigma_v^2,sigma_omega^2,1e4,1e4,1e4,1e4]);

delta_t = 0.1;
tf = 30;
t = 0:delta_t:tf;

true_path = zeros(7,length(t)+1);
true_path(:,1) = state0;


x_t=x0;
y_t=y0;
theta_t=theta0;
x_hat = state0;

path_of_store(:,1)=x_hat;
Sigma_t = Sigma_newSystem;
initial_state = state0;
% can be the v

for i=1:length(t)
    % true trajectory
    v_real = normrnd(mu_v,sigma_v);
    omega_real = normrnd(mu_omega,sigma_omega);
    x_pred = motion_model(x_t,y_t,theta_t,v_real,omega_real,delta_t);
    x_t=x_pred(1);y_t=x_pred(2);theta_t=x_pred(3);
    true_path(:,i+1) = [x_t;y_t;theta_t;landmark1(1);landmark1(2);landmark2(1);landmark2(2)];

    range_real = [];
    bear_real = [];

    for j=1:2
        landx = landmarks(j,1);
        landy = landmarks(j,2);

        range_noNoise = sqrt((landx-x_t)^2+(landy-y_t)^2);
        bear_noNoise = atan2(landy-y_t,landx-x_t)-theta_t;
        % real measurements with noise
        range_real = [range_real,range_noNoise + normrnd(0,delta_r)];
        bear_real = [bear_real,wrapToPi(bear_noNoise + normrnd(0,delta_phi))];
    end
    % measurement in the slam algorithms
    [x_hat,Sigma_t] = myekf_slam(x_hat,Sigma_t,v_real,sigma_v,omega_real,sigma_omega,range_real,bear_real,delta_r,delta_phi,delta_t);

    path_of_store(:,i+1) = x_hat;
end


% plot results.

% the refence traj
r = mu_v/mu_omega;
centerX = x0-r*sin(pi/2);
centerY = y0+r*cos(pi/2);
theta_vec = linspace(0,2*pi,100);
x_ref = centerX+r*cos(theta_vec);
y_ref = centerY+r*sin(theta_vec);

figure(1);
plot(true_path(1,:),true_path(2,:),'k','LineWidth',2);
hold on;
plot(path_of_store(1,:),path_of_store(2,:),'r-.','LineWidth',2);
plot(x0,y0,'kp','MarkerSize',12);
plot(x_ref,y_ref,'bo')
plot(landmarks(:,1), landmarks(:,2), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
title('The path of the EKF-SLAM of the robot.');
legend({'The true path','The EKF-SLAM path','The start point','The reference trajectory','the landmark'},'Location','southwest');
xlabel('X')
ylabel('Y')
hold off;
figure(2);
hold on;

plot(landmarks(:,1), landmarks(:,2), 'bo', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'True Landmarks');

plot(path_of_store(4,end), path_of_store(5,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Estimated Landmarks');
plot(path_of_store(6,end), path_of_store(7,end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
draw_ellipse(path_of_store(4:5,end), Sigma_t(4:5,4:5), 'r');
draw_ellipse(path_of_store(6:7,end), Sigma_t(6:7,6:7), 'r');
axis([0 12 0 12]);
grid on;
function draw_ellipse(center, cov, color)
    [V, D] = eig(cov);
    angle = atan2(V(2,1), V(1,1));
    a = sqrt(D(1,1));
    b = sqrt(D(2,2));
    t = linspace(0, 2*pi, 100);
    ellipse = [a*cos(t); b*sin(t)];
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    ellipse = R * ellipse;
    plot(center(1)+ellipse(1,:), center(2)+ellipse(2,:), color, 'LineWidth', 1);
end