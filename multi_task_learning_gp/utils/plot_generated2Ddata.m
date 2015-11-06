function plot_generated2Ddata(x, Y, ind_kf_train, ind_kx_train)

M = size(Y,2); % number of tasks
N = floor(M/2);
i = 1;
color = hsv(M);

for j= 1: M 
    subplot(2,N,i)
    %plot3(x(:,1),x(:,2), Y(:,j),'x', 'markersize',7,'color',color(j,:));
    tri=delaunay(x(:,1),x(:,2));
    trisurf(tri,x(:,1),x(:,2),Y(:,j),'facecolor',color(j,:)); 
    title(['Task #' num2str(j)])
    grid on
    hold on
    idx = find(ind_kf_train == j);
    plot3(x(ind_kx_train(idx),1),x(ind_kx_train(idx),2),...
        Y(ind_kx_train(idx),j),'ko','markersize',12,'linewidth',3);
    i = i + 1;
end

% for j= 1: M 
%     % figure;
%     %surf(X1,X2,reshape(Y(:,j),size(X1,1),size(X1,2))); 
%     idx = find(ind_kf == j)
%     ind_kx(idx)
%     plot3(x(ind_kx(idx),1),x(ind_kx(idx),2),...
%     Y(ind_kx(idx),j),'o','markersize',18,'color',color(j,:));
%     hold on
%     plot3(x(ind_kx(idx),1),x(ind_kx(idx),2),...
%     Y(ind_kx(idx),j),'rx', 'markersize',7);
%     %plot3(x(:,1),x(:,2), Y(:,j),'rx', 'markersize',7);
%     grid on;
%     pause;
% end

end



