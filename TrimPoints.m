function [pd_t] = TrimPoints(pd, trim_op)
    % TrimPoints.m
    %   Trim a matrix of points so they all lie on the same plane
    %
    % Arguments:
    %   pd: Matrix of points
    %   trim_op: whether to keep points on xz or yz plane ('yz' or 'xz')
    %
    % Returns:
    %   pd_t: Trimmed matrix of points
    
    pd_t = [];
    if trim_op == "xz"
        for i=1:size(pd,2)
            if pd(2,i) == 0
                pd_t = [pd_t, pd(:,i)];
            end
        end
    elseif trim_op == "yz"
        for i=1:size(pd,2)
            if pd(1,i) == 0
                pd_t = [pd_t, pd(:,i)];
            end
        end
    end
end