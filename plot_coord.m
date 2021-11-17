function [] = plot_coord(R,T,scale)
    vx = R*[scale(1,1);0;0];
    vy = R*[0;scale(1,2);0];
    vz = R*[0;0;scale(1,3)];
    px = [T,vx+T];
    py = [T,vy+T];
    pz = [T,vz+T];
    plot3(px(1,:),px(2,:),px(3,:),'red');
    hold on;
    plot3(py(1,:),py(2,:),py(3,:),'green');
    plot3(pz(1,:),pz(2,:),pz(3,:),'blue');
end