function [cineq, c] = OrthCons(x, con_w)
    % OrthConsJac.m
    %   Calculate orthonormality constraint values
    %
    % Arguments:
    %   x: P represented as a design parameter column vector
    %
    % Returns:
    %   c: orthonormality constraint values

    cineq = [];
    
    % In addition to orthonormality, equality constrain intrinsic camera
    % parameters between views
    W_l = [x(1,1), 0, x(3,1);
        0, x(2,1), x(4,1);
        0, 0, 1];
    W_r = [x(17,1), 0, x(19,1);
        0, x(18,1), x(20,1);
        0, 0, 1];
    R_l = reshape(x(5:13,1),[3,3]);
    R_r = reshape(x(21:29,1),[3,3]);
    T_l = x(14:16,1);
    T_r = x(30:32,1);
%     P_l = W{1}*[R_l, T_l];
%     P_r = W{2}*[R_r, T_r];
    
%     c = [norm(R_l(1,:))-1;
%         norm(R_l(2,:))-1;
%         norm(R_l(3,:))-1;
%         cross(R_l(1,:)',R_l(3,:)').*cross(R_l(2,:)',R_l(3,:)');
%         norm(R_r(1,:))-1;
%         norm(R_r(2,:))-1;
%         norm(R_r(3,:))-1;
%         cross(R_r(1,:)',R_r(3,:)').*cross(R_r(2,:)',R_r(3,:)')];
    
    c = [norm(transpose(R_l)*R_l - eye(3,3));
        cross(R_l(1,:)',R_l(3,:)').*cross(R_l(2,:)',R_l(3,:)');
        norm(transpose(R_r)*R_r - eye(3,3));
        cross(R_r(1,:)',R_r(3,:)').*cross(R_r(2,:)',R_r(3,:)')];
    
%     c = [norm(transpose(R_l) - inv(R_l));
%         cross(R_l(1,:)',R_l(3,:)').*cross(R_l(2,:)',R_l(3,:)');
%         norm(transpose(R_r) - inv(R_r));
%         cross(R_r(1,:)',R_r(3,:)').*cross(R_r(2,:)',R_r(3,:)')];
    
    if con_w == true
        c = [c;x(1:4,1)-x(17:20,1)];
    end
end