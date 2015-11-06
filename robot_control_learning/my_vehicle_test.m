% my script

clear all
close all

addpath(genpath('D:\Work\matlab_code\ebonilla-mtgp-da088e8'))
addpath(genpath('D:\Work\matlab_code\gpml-matlab'))

 % covariance V (2x2) matrix corresponding to the odometry vector [dx dtheta].
 % V = [0.1 0.01].^2;
 
 % Options
 % 'stlim',A       Steering angle limited to -A to +A (default 0.5 rad)
 A = 0.5; % rad
 % 'vmax',S        Maximum speed (default 5m/s)
 S = 5; % m/s
 % 'L',L           Wheel base (default 1m)
 L = 1; % 1 m
 % 'x0',x0         Initial state (default (0,0,0) )
 % 'dt',T          Time interval
 T = 0.1;
 % 'rdim',R        Robot size as fraction of plot window (default 0.2)
 R = 0.2;
 
 % 'verbose'       Be verbose
 
 % dim rectangular region
 dim = 10;
 offset = 5;
 a = -dim;
 b = dim; 
 n_samples = 5;
 n_step = 1000;
 
 state = cell(3,n_samples);
 cmd_vel = cell(1,n_samples);
 odom_cov = cell(1,n_samples);
 
 disp('Data collection: START');
 for i = 1 : n_samples
     % create a random covariance matrix for odometry
     V = diag(randn(2,1).^2);
     
     % create a random Initial state
     x0 = (b-a).*rand(3,1) + a;
     % create the bicycle
     v = Vehicle(V,'stlim',A,'vmax',S,'L',L,'dt',T,'rdim',R);
     v.init(x0);
     % create the driver
     driver = RandomPath(dim + offset);
     % display(driver)
     v.add_driver(driver);  
     % v.driver.visualize();
     
     u =  zeros(n_step,2);
     true_xnext = zeros(n_step + 1,3);
     noisy_xnext = zeros(n_step + 1,3);
     
     true_xnext(1,:) = x0;
     noisy_xnext(1,:) = x0;
     v.x_hist = [v.x_hist; x0'];
     
     for j = 1 : n_step
        % v.visualize();
        [speed, steer] = v.driver.demand();
        % clip the speed
        u(j,1) = min(v.maxspeed, max(-v.maxspeed, speed));
        % clip the steering angle
        u(j,2) = max(-v.alphalim, min(v.alphalim, steer));
        xp = v.x; % previous state
        % pause
        v.x(1) = v.x(1) + u(j,1)*v.dt*cos(v.x(3));
        v.x(2) = v.x(2) + u(j,1)*v.dt*sin(v.x(3));
        v.x(3) = v.x(3) + u(j,1)*v.dt/v.L * u(j,2);
        odo = [colnorm(v.x(1:2)-xp(1:2)) v.x(3)-xp(3)];
        v.odometry = odo;
        true_xnext(j+1,:) = v.f(v.x', v.odometry);
        v.x_hist = [v.x_hist; v.x'];   % maintain history
        odo = v.odometry + randn(1,2)*v.V;
        noisy_xnext(j+1,:) = v.f(v.x', v.odometry,v.V);
        % v.plot(); 
        % hold on
        %plot(true_xnext(j,1),true_xnext(j,2),'-r','linewidth',3);
        % plot(noisy_xnext(j,1),noisy_xnext(j,2),'*r','linewidth',10);
        %hold on
        %plot(noisy_xnext(j,1),noisy_xnext(j,2),'-k','linewidth',3);
        % plot(xp(1),xp(2),'.k','linewidth',1);
        %drawnow
     end
     
     state{1,i} = true_xnext;
     state{2,i} = noisy_xnext;
     state{3,i} = v.x_hist;
     cmd_vel{i} = u;
     odom_cov{i} = V;
     
%      p = v.run(n_step)
%      state{i} = p;
%        v.plot_xy();
%        pause
%        hold on
%        plot(true_xnext(:,1),true_xnext(:,2),'-r','linewidth',3);
%        pause
%        plot(noisy_xnext(:,1),noisy_xnext(:,2),'-k');
%        hold off
%        pause
 end
disp('Data collection: END');
disp('Data orgaization for learning: START');
x = [];
y = [];
for i = 1 : n_samples
    noisy_xnext = state{2,i};
    cmd = cmd_vel{i};
    temp = [noisy_xnext(1:end-1,:),cmd];
    x = [x;temp];
    y = [y;noisy_xnext(2:end,:)];
end

M = size(y,2); % Number of tasks
[N,D] = size(x); % Num of samples + dimensionality of input space
y = y(:);
v  = repmat((1:M),N,1);
ind_kf = v(:);
v  = repmat((1:N)',1,M);
ind_kx = v(:); 
Y  = reshape(y,N,M);
n = N*M;
p = 0.3;
ntrain       = floor(p*n);
v            = randperm(n); 
idx_train    = v(1:ntrain);
nx           = ones(ntrain,1); % observations on each task-input point
ytrain       = y(idx_train);
xtrain       = x; 
ind_kx_train = ind_kx(idx_train);
ind_kf_train = ind_kf(idx_train);
covfunc_x       = {'covSEard'};
irank           = M;       % rank for Kf (1, ... M). irank=M -> Full rank
data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};
[logtheta_all deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);
disp('Data orgaization for learning: END');

disp('Learning: START');

[logtheta_all nl]           = learn_mtgp(logtheta_all, deriv_range, data);
disp('Learning: END');

disp('Prediction: START');
[ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );
disp('Prediction: END');

%  figure;
%  true_xnext = state{1,1};
%  noisy_xnext = state{2,1};
%  history = state{3,1};
%  cmd = cmd_vel{1};
%  subplot(131)
%  plot(true_xnext(:,1),true_xnext(:,2),'color','r','linewidth',3);
%  grid on
%  subplot(132) 
%  plot(noisy_xnext(:,1),noisy_xnext(:,2),'color','k','linewidth',3);
%  grid on
%  subplot(133)
%  plot(history(:,1),history(:,2),'color','m','linewidth',3);
%  grid on
 
% x = [noisy_xnext(1:end-1,:),cmd];
% y = noisy_xnext(2:end,:);
% y = y(:);
% M = 3; % Number of tasks
% [N,D] = size(x); % Num of samples + dimensionality of input space
% v  = repmat((1:M),N,1);
% ind_kf = v(:);
% v  = repmat((1:N)',1,M);
% ind_kx = v(:); 
% Y  = reshape(y,N,M);
% n = N*M;
% 
% p = 0.3;
% ntrain       = floor(p*n);
% v            = randperm(n); 
% idx_train    = v(1:ntrain);
% nx           = ones(ntrain,1); % observations on each task-input point
% ytrain       = y(idx_train);
% xtrain       = x; 
% ind_kx_train = ind_kx(idx_train);
% ind_kf_train = ind_kf(idx_train);
% 
% save('bicycle-data.mat','x', 'Y', 'xtrain', 'ytrain', 'ind_kf_train', 'ind_kx_train', 'nx');