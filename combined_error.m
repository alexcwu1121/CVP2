function [error] = combined_error(x, p3d, p2d_arr)
    error = 0;
    for i=1:size(p2d_arr,2)
        W = [x(1+16*(i-1),1), 0, x(3+16*(i-1),1);
            0, x(2+16*(i-1),1), x(4+16*(i-1),1);
            0, 0, 1];
        R = reshape(x(5+16*(i-1):13+16*(i-1),1),[3,3]);
        T = x(14+16*(i-1):16+16*(i-1),1);
        p = W*[R, T];
        error = error + ReprojError(p, p3d, p2d_arr{i}, false);
    end
    error = error^2;
end