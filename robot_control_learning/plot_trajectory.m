function plot_trajectory( state,flag,num_traj)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% flags = 1 -> true trajectories
% flags = 2 -> noisy trajectories

[n,m] = size(state);

if(flag <n && num_traj <=m)

    trajectory = state{flag,num_traj};
    x = trajectory(:,1);
    y =  trajectory(:,2);
    %theta = trajectory(:,3);
    plot(x,y);
else
    disp('Error');
end

end

