function [] = plot_points(p3d, p2d_l, p2d_r)
    figure(1);
    scatter3(p3d(1,:),p3d(2,:),p3d(3,:));
    title("3D Points");
    figure(2);
    scatter(p2d_l(1,:),p2d_l(2,:));
    title("2D Points from 'Left' Perspective");
    figure(3);
    scatter(p2d_r(1,:),p2d_r(2,:));
    title("2D Points from 'Right' Perspective");
end