function [ xtrain, ytrain,x,Y, M, nx, ind_kf_train, ind_kx_train ] = get_training_data(state,cmd_vel,n_samples,p)
disp('Data orgaization for learning');
x = [];
y = [];
for i = 1 : n_samples
    noisy_xnext = state{2,i};
    cmd = cmd_vel{i};
    temp = [noisy_xnext(1:end-1,:),cmd];
    x = [x;temp];
    y = [y;noisy_xnext(2:end,:)];
end
% Number of tasks
M = size(y,2); 
% Num of samples + dimensionality of input space
[N,D] = size(x); 

y = y(:);
v  = repmat((1:M),N,1);
ind_kf = v(:);
v  = repmat((1:N)',1,M);
ind_kx = v(:); 
Y  = reshape(y,N,M);
n = N*M;

ntrain       = floor(p*n);
v            = randperm(n); 
idx_train    = v(1:ntrain);
nx           = ones(ntrain,1); % observations on each task-input point
ytrain       = y(idx_train);
xtrain       = x; 
ind_kx_train = ind_kx(idx_train);
ind_kf_train = ind_kf(idx_train);

end

