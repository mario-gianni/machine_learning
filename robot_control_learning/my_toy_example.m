% my_toy_exmaple
clear all
close all

covfunc_x       = {'covSEard'};
M               = 6;       % Number of tasks
D               = 2;       % Dimensionality of input spacce
irank           = M;       % rank for Kf (1, ... M). irank=M -> Full rank
n_points        = 70;      % Number of samples to be generated
rho             = 0.1;     % Correlations between problems
sigma_2n        = 0.001;   % Noise variance
min_range       = -2;      % Min grid value
max_range       = 2;       % Max grid value

%% 1. Generating samples from true Model
[x, Y, xtrain, ytrain, ind_kf_train, ind_kx_train, nx] = my_generate_data(covfunc_x,M,min_range,max_range,n_points,rho,sigma_2n);

plot_generated2Ddata(x, Y, ind_kf_train, ind_kx_train);

save('data.mat','x', 'Y', 'xtrain', 'ytrain', 'ind_kf_train', 'ind_kx_train', 'nx');

%% 2. Assigns cell data for learning and prediction
data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};

%% 3. Hyper-parameter learning
 [logtheta_all deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);
 
 [logtheta_all nl]           = learn_mtgp(logtheta_all, deriv_range, data);
 
 %% 4. Making predictions at all points on all tasks
[ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );