function [rp] = rectify_r(R, T)
    vx_l = R*[1;0;0];
    base_l = T;
    %theta = acos(vx_l*base_l);
    theta_l = acos(dot(vx_l,base_l/norm(base_l)));
    axis_l = cross(transpose(vx_l),transpose(base_l));
    rp = axang2rotm([axis_l, theta_l]);
end