function [error] = combined_error(x, p3d, p2d_arr, W)
    error = 0;
    for i=1:size(p2d_arr,2)
        R = reshape(x(1+12*(i-1):9+12*(i-1),1),[3,3]);
        T = x(10+12*(i-1):12+12*(i-1),1);
        p = W{i}*[R, T];
        error = error + ReprojError(p, p3d, p2d_arr{i}, false);
    end
    error = error^2;
end