function [error] = ReprojError(P, p3d, p2d, display)
    % ReprojError.m
    %   Calculate reprojection error
    %
    % Arguments:
    %   x: P represented as a design parameter column vector
    %   p3d: matrix of 3D points
    %   p2d: matrix of 2D points
    %   display: whether or not to display intermediates in ReprojError
    %
    % Returns:
    %   error: reprojection error

    error = 0;
    for i=1:size(p3d,2)
        lhs = [p2d(:,i);1]';
        rhs = [(1/P(3,4))*P*[p3d(:,i);1]]';
        if display
            disp([lhs rhs]);
        end
        diff = lhs - rhs;
        error = error + diff*diff';
        %error = error + norm(diff)^2;
    end
end