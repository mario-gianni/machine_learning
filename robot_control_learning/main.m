clear all
close all

addpath(genpath('D:\Work\LocalProjects\machine_learning\multi_task_learning_gp'));
addpath(genpath('D:\Work\LocalProjects\machine_learning\gaussian_processes'));

n_samples = 10;
n_step = 1000;

plot = 0;

[state,cmd_vel,odom_cov] = generate_vehicle_data(n_samples,n_step,plot);

p = 0.3;

[ xtrain,ytrain,x,Y,M,nx,ind_kf_train,ind_kx_train] = get_training_data(state,cmd_vel,n_samples,p);

% covfunc_x = {'covSEard'};
covfunc_x = {'covSum', {'covSEard','covNoise'}};
irank = M; % rank for Kf (1, ... M). irank=M -> Full rank
data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};
[logtheta_all,deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);
disp('Learning');
[logtheta_all,nl] = learn_mtgp(logtheta_all, deriv_range, data);
disp('Prediction');
[ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );
disp('Finished');
% load control-data.mat
% 
% M = 6; % number of tasks
% [N D] = size(x); % size of input
% D = 6; % Dimensionality of input space
% covfunc_x = {'covSEard'};
% irank = M;    % rank for Kf (1, ... M). irank=M -> Full rank
% data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};
% 
% [logtheta_all,deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);
% 
% [logtheta_all,nl] = learn_mtgp(logtheta_all, deriv_range, data);
% 
% [ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );
% 
% plot(nl)
% title('Negative Marginal log-likelihood');