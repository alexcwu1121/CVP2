function [a_row] = RowA(M, m)
    % RowA.m
    %   Assemble two rows of A matrix from a point pair
    % 
    % Arguments:
    %   M: 3D point
    %   m: 2D point
    %
    % Returns:
    %   a_row: two rows of A

    a_row = [transpose(M), 1, 0, 0, 0, 0, -m(1,1)*transpose(M), -m(1,1);
        0, 0, 0, 0, transpose(M), 1, -m(2,1)*transpose(M), -m(2,1)];
end