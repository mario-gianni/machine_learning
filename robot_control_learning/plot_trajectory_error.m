function plot_trajectory_error( state,num_traj )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

[n,m] = size(state);

if(num_traj <=m)

    trajectory_ref = state{1,num_traj};
    trajectory_real = state{2,num_traj};
    %error = sqrt((trajectory_ref - trajectory_real).^2);
    error_x = (diff(trajectory_ref(:,1)-trajectory_real(:,1))).^2;
    error_y = (diff(trajectory_ref(:,2)-trajectory_real(:,2))).^2;
    error_theta = (diff(trajectory_ref(:,3)-trajectory_real(:,3))).^2;
    error = error_x + error_y; 
    plot(error);
    %hold on
    %plot(error_y);
    %plot(error_theta);
    %hold off
else
    disp('Error');
end


end

