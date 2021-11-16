function [cineq, c] = OrthCons(x, W, con_w)
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
    R_l = reshape(x(1:9,1),[3,3]);
    R_r = reshape(x(13:21,1),[3,3]);
    T_l = x(10:12,1);
    T_r = x(22:24,1);
%     P_l = W{1}*[R_l, T_l];
%     P_r = W{2}*[R_r, T_r];
    
    c = [norm(R_l(3,:))-1;
        cross(R_l(1,:)',R_l(3,:)').*cross(R_l(2,:)',R_l(3,:)');
        norm(R_r(3,:))-1;
        cross(R_r(1,:)',R_r(3,:)').*cross(R_r(2,:)',R_r(3,:)')];
    
    if con_w == true
        %[W_l, ~, ~] = recover_params(P_l);
        %[W_r, ~, ~] = recover_params(P_r);

        diff = W{1} - W{2};
        c = [c;diff(1,1);diff(2,2);diff(1,3);diff(2,3);];
    end
end