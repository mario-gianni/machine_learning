function [ state,cmd_vel,odom_cov ] = generate_vehicle_data( n_samples,n_step, plot_)

disp('Data collection');

% Vehicle properties
% Steering angle limited to -A to +A (default 0.5 rad)
A = 0.5; % rad
% Maximum speed (default 5m/s)
S = 5;
% Wheel base (default 1m)
L = 1;
% Time interval
T = 0.1;
% Robot size as fraction of plot window (default 0.2)
R = 0.2;

% Environment properties
% dim rectangular region
dim = 10;
offset = 5;
a = -dim;
b = dim;

% Output values
state = cell(3,n_samples);
cmd_vel = cell(1,n_samples);
odom_cov = cell(1,n_samples);

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
     if(plot_ == 1)
        display(driver);
     end
     v.add_driver(driver);  
     if(plot_ == 1)
        v.driver.visualize();
     end
     u =  zeros(n_step,2);
     true_xnext = zeros(n_step + 1,3);
     noisy_xnext = zeros(n_step + 1,3);
     
     true_xnext(1,:) = x0;
     noisy_xnext(1,:) = x0;
     v.x_hist = [v.x_hist; x0'];
     
     for j = 1 : n_step
         if (plot_ == 1)
            v.visualize();
         end
        [speed, steer] = v.driver.demand();
        % clip the speed
        u(j,1) = min(v.maxspeed, max(-v.maxspeed, speed));
        % clip the steering angle
        u(j,2) = max(-v.alphalim, min(v.alphalim, steer));
        xp = v.x; % previous state
        v.x(1) = v.x(1) + u(j,1)*v.dt*cos(v.x(3));
        v.x(2) = v.x(2) + u(j,1)*v.dt*sin(v.x(3));
        v.x(3) = v.x(3) + u(j,1)*v.dt/v.L * u(j,2);
        odo = [colnorm(v.x(1:2)-xp(1:2)) v.x(3)-xp(3)];
        v.odometry = odo;
        true_xnext(j+1,:) = v.f(v.x', v.odometry);
        v.x_hist = [v.x_hist; v.x'];   % maintain history
        noisy_xnext(j+1,:) = v.f(v.x', v.odometry,v.V);
        if(plot_ == 1) 
            v.plot();
            drawnow
        end
     end
     if (plot_ == 1)
        v.plot_xy();
        pause(1);
     end
     
     state{1,i} = true_xnext;
     state{2,i} = noisy_xnext;
     state{3,i} = v.x_hist;
     cmd_vel{i} = u;
     odom_cov{i} = V;
     
 end

end

