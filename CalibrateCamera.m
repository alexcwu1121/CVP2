% What happens when you:
%   Choose 6 coplanar points?: Rank is less than 11. AssembleA never
%       finishes
%   Choose 3 points from each plane?: 
function [P] = CalibrateCamera(p3d, p2d, n_yz, n_xz, random, unique_op, W_supply)
    % CalibrateCamera.m
    %   1. Randomly sample 6 3D/2D point pairs, either by # of pairs per
    %   plane or pure random (IE: 2 from yz plane and 4 from xz plane)
    %   2. Repeat step 1 until an A with rank=11 is produced from samples
    %   3. Find nullspace of A (V)
    %   4. Normalize V (norm(p3)=1)
    %   5. Decompose V into P
    %
    % Arguments:
    %   p3d: matrix of 3D points
    %   p2d: matrix of 2D points
    %   n_yz: number of samples to take from yz plane
    %   n_xz: number of samples to take from xz plane
    %   random: boolean whether sampling should be pure random
    %   unique_op: select only "unique" points
    %       3D points from yz plane should have no repeated y or z
    %       3D points from xz plane should have no repeated x or z
    %
    % Returns:
    %   P: projection matrix
    %
    % Observations:
    %   When 5 points are taken from yz plane and 1 from xz or vice versa,
    %       reprojection error is high (e+13). Why?
    %   The lowest reprojection errors are seen when 3 samples are taken from
    %       each plane. Why?
    %   If sampled points are all unique, rank of A is 12 and no solution
    %       can be found. Why?
    while true
        [p3d_sample, p2d_sample] = SamplePoints(p3d, p2d, n_yz,...
            n_xz, random, unique_op);
        
        % if an intrinsic camera matrix is provided, solve only for
        % extrinsic parameters
        if size(W_supply) ~= 0
            %p2d_transform = inv(W_supply)*[p2d_sample;1];
            A = AssembleAW(p3d_sample, p2d_sample, W_supply);
        else
            A = AssembleA(p3d_sample, p2d_sample);
        end
        
        % Full rank has no nullspace and rank <11 does not have unique solution
        if rank(A) == 11
            break;
        end
    end
    %figure(3);
    %disp(p3d_sample);
    %scatter3(p3d_sample(1,:),p3d_sample(2,:),p3d_sample(3,:));

    % Find null space of A
    % disp("Nullspace of A");
    Vinit = null(A);
    % disp(Vinit);
    % magnitude of p3 must be 1 due to orthonormality
    % Divide V by magnitude of V(9:11,1)
    % disp("Orthonormality: normalized V");
    Vinit = Vinit/norm(Vinit(9:11,1));
    % disp(Vinit);

    % Assemble projection matrix and evaluate average projection error
    P = [transpose(Vinit(1:4,1));
        transpose(Vinit(5:8,1));
        transpose(Vinit(9:12,1))];
    %disp(P);
end