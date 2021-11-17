function [error] = obj_recon(x, R, T, W_l, W_r, p_l, p_r)
    % lambdal, lambdar, xl, yl, zl
    e1 = norm(x(1,1)*[p_l;1] - W_l*x(3:5,1))^2;
    e2 = norm(x(2,1)*[p_r;1] - W_r*(transpose(R)*x(3:5,1)-transpose(R)*T))^2;
    error = e1 + e2;
end