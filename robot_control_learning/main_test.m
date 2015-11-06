clear all
close all

n_points = 4;

min_track_vel = -0.6; % m/s
max_track_vel = 0.6; % m/s

track_vel_range = linspace(min_track_vel,max_track_vel,n_points)';
% [vr,vl] = meshgrid(track_vel_range, track_vel_range);
% cmd_vel = [vr(:), vl(:)];

%n_points = 5;
min_flipper_angle = -2; % radians
max_flipper_angle = 2;
flipper_pos_range = linspace(min_flipper_angle,max_flipper_angle,n_points)';
[vr,vl,a1,a2,a3,a4] = ndgrid(track_vel_range,track_vel_range,flipper_pos_range,flipper_pos_range,flipper_pos_range,flipper_pos_range);
%flippers_cmd = [a1(:),a2(:),a3(:),a4(:)];

x = [vr(:), vl(:),a1(:),a2(:),a3(:),a4(:)];

M = 6; % number of tasks
[N D] = size(x);
D = 6; % Dimensionality of input space

rho = 0.8; % Correlations between problems
%% Task covariance
Kf = rho*ones(M);
idx_diag = sub2ind(size(Kf), 1:M, 1:M);
Kf(idx_diag) = 1;

covfunc_x = {'covSEard'};

%% Input covariance
ltheta_x =  eval(feval(covfunc_x{:}));
theta_x = log(ones(ltheta_x,1));      

sigma_2n = 0.0001; % Noise variance

%% Noise variances (re-parametrized)
theta_sigma = log(sqrt(sigma_2n*ones(M,1))); 
%% Full covariance
Sigma2 = diag(exp(2*theta_sigma));
Kx = feval(covfunc_x{:}, theta_x, x);
C = kron(Kf,Kx) + kron(Sigma2,eye(N));
L = chol(C)';    

%% Observations
n = N*M;
u = randn(n,1);
y = L*u;
v      = repmat((1:M),N,1); 
ind_kf = v(:);  % indices to task
v      = repmat((1:N)',1,M);
ind_kx = v(:);           % indices to input space data-points
Y      = reshape(y,N,M); % Matrix of observations across all tasks


%% Selecting data-points for training
ntrain       = floor(0.1*n);
v            = randperm(n); 
idx_train    = v(1:ntrain);
nx           = ones(ntrain,1); % observations on each task-input point
ytrain       = y(idx_train);
xtrain       = x; 
ind_kx_train = ind_kx(idx_train);
ind_kf_train = ind_kf(idx_train);

save('control-data.mat','x', 'Y', 'xtrain', 'ytrain', 'ind_kf_train', 'ind_kx_train', 'nx');