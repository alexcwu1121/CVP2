function [p3d_sample, p2d_sample] = SamplePoints(p3d, p2d,...
    yz_n, xz_n, random, unique_op)
    % SamplePoints.m
    %   Randomly sample point pairs either completely randomly or
    %   stratified between yz and xz plane
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
    %   p3d_sample: sampled 3D points
    %   p2d_sample: sampled 2D points

    if random
        sample_ind = randi([1 size(p3d,2)],1,yz_n+xz_n);
        p3d_sample = p3d(:,sample_ind);
        p2d_sample = p2d(:,sample_ind);
    else
%         if unique_op
%             [~,idx] = unique(p3d(2:3,:)','rows');
%             p3d_u = p3d(:,idx)';
%             disp(p3d);
%             disp(p3d_u);
%             disp(">>>>");
%             [~,idx] = unique(p3d_u(:,3),'rows');
%             p3d_u = p3d_u(idx);
%              %disp(p3d_u);
%         end
        pd_t = TrimPoints([p3d;p2d], 'yz');
        p3d_yz = pd_t(1:3,:);p2d_yz = pd_t(4:5,:);

        pd_t = TrimPoints([p3d;p2d], 'xz');
        p3d_xz = pd_t(1:3,:);p2d_xz = pd_t(4:5,:);
        
        sample_p3d_yz=[];
        sample_p3d_xz=[];
        sample_p2d_yz=[];
        sample_p2d_xz=[];
        while size(sample_p3d_yz,2) < yz_n
            idx=randi([1 size(p3d_yz,2)],1,1);
            
            if unique_op
                if isempty(sample_p3d_yz) ...
                        || (~ismember(p3d_yz(2,idx), sample_p3d_yz(2,:)) && ...
                        ~ismember(p3d_yz(3,idx), sample_p3d_yz(3,:)))
                    sample_p3d_yz=[sample_p3d_yz, p3d_yz(:,idx)];
                    sample_p2d_yz=[sample_p2d_yz, p2d_yz(:,idx)];
                end
            else
                sample_p3d_yz=[sample_p3d_yz, p3d_yz(:,idx)];
                sample_p2d_yz=[sample_p2d_yz, p2d_yz(:,idx)];
            end
        end
        while size(sample_p3d_xz,2) < xz_n
            idx=randi([1 size(p3d_xz,2)],1,1);
            
            if unique_op
                if isempty(sample_p3d_xz) ...
                        || (~ismember(p3d_xz(1,idx), sample_p3d_xz(1,:)) && ...
                        ~ismember(p3d_xz(3,idx), sample_p3d_xz(3,:)))
                    sample_p3d_xz=[sample_p3d_xz, p3d_xz(:,idx)];
                    sample_p2d_xz=[sample_p2d_xz, p2d_xz(:,idx)];
                end
            else
                sample_p3d_xz=[sample_p3d_xz, p3d_xz(:,idx)];
                sample_p2d_xz=[sample_p2d_xz, p2d_xz(:,idx)];
            end
        end

        p3d_sample = [sample_p3d_yz,sample_p3d_xz];
        p2d_sample = [sample_p2d_yz,sample_p2d_xz];
    end
end