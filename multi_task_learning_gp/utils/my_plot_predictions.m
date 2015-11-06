function my_plot_predictions(x, Y, Ypred, ind_kf_train, ind_kx_train)
M = size(Y,2);
N = floor(M/2);
i = 1;
color = hsv(M);
    

% for j= 1: M 
%     subplot(2,N,i)
%     plot3(x(:,1),x(:,2), Y(:,j),'x', 'markersize',7,'color',color(j,:));
%     %surf(X1,X2,reshape(Y(:,j),size(X1,1),size(X1,2))); 
%     idx = find(ind_kf_train == j);
%     hold on;
%     plot3(x(ind_kx_train(idx),1),x(ind_kx_train(idx),2),...
%     Y(ind_kx_train(idx),j),'kx','markersize',7,'linewidth',3);
%     %hold on;
%     %plot3(x(:,1),x(:,2),Ypred(:,j),'ko','markersize',12,'linewidth',3);  
%     grid on;
%     i = i + 1;
% end
i = 0;
for j= 1: 2 
    
    subplot(M,3,j+i)
    grid on;
    tri=delaunay(x(:,1),x(:,2));
    trisurf(tri,x(:,1),x(:,2),Y(:,j),'facecolor',color(j,:));
    i = i + 1;
    
    subplot(M,3,j+i)
    grid on;
    idx = find(ind_kf_train == j);
    tri=delaunay(x(ind_kx_train(idx),1),x(ind_kx_train(idx),2),Y(ind_kx_train(idx),j));
    trisurf(tri,x(ind_kx_train(idx),1),x(ind_kx_train(idx),2),Y(ind_kx_train(idx),j),'facecolor',color(j,:))
    i = i + 1;
    
    subplot(M,3,j+i)
    %plot3(x(:,1),x(:,2), Y(:,j),'ko', 'markersize',7,'color',color(j,:));
    %surf(X1,X2,reshape(Y(:,j),size(X1,1),size(X1,2))); 
    %idx = find(ind_kf_train == j);
    %hold on;
    %plot3(x(ind_kx_train(idx),1),x(ind_kx_train(idx),2),...
    %Y(ind_kx_train(idx),j),'kx','markersize',7,'linewidth',3);
    %hold on;
    tri=delaunay(x(:,1),x(:,2),Ypred(:,j));
    trisurf(tri,x(:,1),x(:,2),Ypred(:,j),'facecolor',color(j,:));
    grid on;
 
end

end



