% state estimation
clear all
close all
addpath(genpath('D:\Work\matlab_code\ebonilla-mtgp-da088e8'))
addpath(genpath('D:\Work\matlab_code\gpml-matlab'))
load bicycle-data.mat
covfunc_x       = {'covSEard'};
M               = 3;       % Number of tasks
D               = 5;       % Dimensionality of input spacce
irank           = M;       % rank for Kf (1, ... M). irank=M -> Full rank
data  = {covfunc_x, xtrain, ytrain, M, irank, nx, ind_kf_train, ind_kx_train};
[logtheta_all deriv_range] = init_mtgp_default(xtrain, covfunc_x, M, irank);
[logtheta_all nl]           = learn_mtgp(logtheta_all, deriv_range, data);
[ Ypred, Vpred ] = predict_mtgp_all_tasks(logtheta_all, data, x );

error_pos = sqrt((Y(:,1) - Ypred(:,1)).^2 + (Y(:,2) - Ypred(:,2)).^2);
error_orient = abs(Y(:,3) - Ypred(:,3));

plot(Y(:,1),Y(:,2),'ob','MarkerSize', 5);
grid on
hold on 
plot(Ypred(:,1),Ypred(:,2),'+m','MarkerSize', 5);
hold off

figure
plot(Y(:,3),'.r')
gid on
hold on
plot(Ypred(:,3),'.m');
hold off


