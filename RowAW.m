function [a_row] = RowAW(M, m, W_supply)
    % RowA.m
    %   Assemble two rows of A matrix from a point pair
    % 
    % Arguments:
    %   M: 3D point
    %   m: 2D point
    %
    % Returns:
    %   a_row: two rows of A
    m_t = inv(W_supply)*[m;1];
    C = 1/m_t(3,1);

    a_row = [transpose(M), 1, 0, 0, 0, 0, -C*m_t(1,1)*transpose(M), -C*m_t(1,1);
        0, 0, 0, 0, transpose(M), 1, -C*m_t(2,1)*transpose(M), -C*m_t(2,1)];
end