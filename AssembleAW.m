function [A] = AssembleAW(p3d, p2d, W_supply)
    % AssembleA.m
    %   Build A matrix given a set of point pairs
    %
    % Arguments:
    %   p3d: matrix of 3D points
    %   p2d: matrix of 2D points
    %
    % Returns:
    %   A: A matrix

    % For each point within a sample, add two rows to A matrix
    A = [];
    for i=1:size(p3d,2)
        A = [A; RowAW(p3d(:,i), p2d(:,i), W_supply)];
    end
end