function [W, R, T] = recover_params(P)
    r3 = P(3,1:3);
    tz = P(3,4);
    co = P(1,1:3)*transpose(P(3,1:3));
    ro = P(2,1:3)*transpose(P(3,1:3));
    sxf = (P(1,1:3)*transpose(P(1,1:3))-co^2)^(1/2);
    syf = (P(2,1:3)*transpose(P(2,1:3))-ro^2)^(1/2);
    tx = (P(1,4)-co*tz)/sxf;
    ty = (P(2,4)-ro*tz)/syf;
    r1 = (P(1,1:3)-co*P(3,1:3))/sxf;
    r2 = (P(2,1:3)-ro*P(3,1:3))/syf;
    
    R = [r1; r2; r3];
    T = [tx; ty; tz];
    W = [sxf, 0, co;
        0, syf, ro;
        0, 0, 1];
    
    % Verifications
    % R must be orthonormal
%     disp("Orthonormality of R:");
%     disp(norm(r1));
%     disp(norm(r2));
%     disp(norm(r3));
%     disp(cross(r1,r3).*cross(r2,r3));
end