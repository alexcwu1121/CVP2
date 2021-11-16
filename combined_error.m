function [error] = combined_error(x, p3d, p2d_arr, W)
    error = 0;
    for i=1:size(p2d_arr,2)
        R = rotationVectorToMatrix(x(1+6*(i-1):3+6*(i-1),1));
        p = W{i}*[R, x(4+6*(i-1):6+6*(i-1),1)];
        error = error + ReprojError(p, p3d, p2d_arr{i}, false);
    end
    error = error^2;
end