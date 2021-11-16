function [] = plot_project()
    figure(2);
    hold on;
    p2rep_l = [];
    for i=1:size(p3d,2)
        p2rep_l = [p2rep_l,(1/P_l(3,4))*P_l*[p3d(:,i);1]];
    end
    scatter(p2rep_l(1,:),p2rep_l(2,:),'x','red');
end