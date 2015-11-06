clear all
close all

load control-data.mat

M = 6; % number of tasks
[N D] = size(x); % size of input
D = 6; % Dimensionality of input space
covfunc_x = {'covSEard'};
irank = M;    % rank for Kf (1, ... M). irank=M -> Full rank
data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};

[logtheta_all,deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);

[logtheta_all,nl] = learn_mtgp(logtheta_all, deriv_range, data);

[ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );

plot(nl)
title('Negative Marginal log-likelihood');