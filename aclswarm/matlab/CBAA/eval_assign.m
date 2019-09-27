function [dist_old, dist_new] = eval_assign(assign,tau,theta,old_assign,tau0,theta0, p_i,x_i, VISUALIZE)
y_i_new = Rot(theta)*(p_i(:,assign)-mean(p_i,2))+mean(p_i,2)+tau;
y_i_old = Rot(theta0)*(p_i(:,old_assign)-mean(p_i,2))+mean(p_i,2)+tau0;

old = y_i_old-x_i;
new = y_i_new-x_i;
dist_old = sum(sqrt(old(1,:).^2+old(2,:).^2));
dist_new = sum(sqrt(new(1,:).^2+new(2,:).^2));

% line_fh = cell(1,size(x_i,2));
% for i=1:size(x_i,2)
%     line_fh = plot(zeros(2,1), zeros(2,1),'k')
% end

if VISUALIZE == true
    text_cell = {};
    for i=1:numel(assign)
       text_cell = [text_cell, num2str(i)]; 
    end
    
    % visualize old and new alignment
    b_max = max([x_i,y_i_new,y_i_old, p_i]');
    b_min = min([x_i,y_i_new,y_i_old, p_i]');
    dx = 0.1; dy = 0.1; 
    
    fh = figure;
    scatter(y_i_old(1,:),y_i_old(2,:),'sb','DisplayName','aligned formation'); hold on
    scatter(x_i(1,:),x_i(2,:),'k','filled','DisplayName','agent positions')
    % add line connecting points
    plot([x_i(1,:);y_i_old(1,:)], [x_i(2,:);y_i_old(2,:)])
    legend

    text(y_i_old(1,:)+dx,y_i_old(2,:)+dy,text_cell,'Color','b','FontSize', 14);
    text(x_i(1,:)-dx,x_i(2,:)-dy,text_cell,'Color','k','FontSize', 14);
    axis([b_min(1) b_max(1) b_min(2) b_max(2)]+[-1 1 -1 1])
    set(fh,'Position',[0,500,500,400])
    
    fh = figure;
    scatter(y_i_new(1,:),y_i_new(2,:),'sr','DisplayName','alignment and reassignment'); hold on
    scatter(x_i(1,:),x_i(2,:),'k','filled','DisplayName','agent positions')
    % add line connecting points
    plot([x_i(1,:);y_i_new(1,:)], [x_i(2,:);y_i_new(2,:)])
    legend
    text(y_i_new(1,:)+dx,y_i_new(2,:)+dy,text_cell,'Color','r','FontSize', 14);
    text(x_i(1,:)-dx,x_i(2,:)-dy,text_cell,'Color','k','FontSize', 14);
    axis([b_min(1) b_max(1) b_min(2) b_max(2)]+[-1 1 -1 1])
    set(fh,'Position',[600,500,500,400])
        
end
end