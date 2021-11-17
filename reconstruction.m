clear all; close all;

% This is an example of a case in which intrinsic parameters are not
% constrained.
% W_l = 1.0e+03 * [2.1612,0,1.3746;
%     0,2.4709,0.7002;
%     0,0,0.0010];
% 
% R_l = [-0.8915,0.4530,0.0000;
%     -0.0000,-0.0000,1.0000;
%     0.4530,0.8915,0.0000];
% 
% T_l=[-18.2962;
%     -26.6903;
%     209.0204];
% 
% W_r = 1.0e+03 * [2.9584, 0, 0.5439;
%     0,2.9571,0.4574;
%     0,0,0.0010];
% 
% R_r = [-0.9907,0.1364,0.0000;
%     -0.0000,-0.0000,1.0000;
%     0.1364,0.9907,0.0000];
% 
% T_r = [60.6114;
%     5.0357;
%     233.5393];

% Take intrinsic and extrinsic parameters from best calibration run
W_l = 1.0e+03 * [2.5813, 0, -0.3326;
                0, 2.6064, 0.4478;
                0, 0, 0.0010];
R_l = [-0.4444, 0.8958, -0.0000;
    -0.0000, -0.0000, 1.0000;
    0.8958, 0.4444, 0.0000];
T_l = [128.1482;
    -4.1868;
    216.5456];

W_r = W_l;
R_r = [0.8752, -0.4838, -0.0000;
    0.0000, 0.0000, -1.0000;
    -0.4838, -0.8752, -0.0000];
T_r = [-131.2874;
    -5.4914;
    -207.4080];

disp("rots");
disp(R_l*R_r);

R_prime = R_l*transpose(R_r);
T_prime = T_l-R_prime*T_r;

p2d_lface = ParseMat("resources/pts_left.txt", 2, 0);
p2d_rface = ParseMat("resources/pts_right.txt", 2, 0);

x0 = [1;1;1;1;1];

% Solve nonlinear system of eqs for each point correspondence
% Optimize projection matrices 'Display','iter'
p3d_recon = [];
for i=1:size(p2d_lface,2)
    options = optimoptions('fmincon','Algorithm','interior-point',...
                            'Display','iter');
    x = fmincon(@(x)obj_recon(x, R_prime, T_prime, W_l, W_r, p2d_lface(:,i), p2d_rface(:,i)),x0,...
            [],[],[],[],[],[],[],options);
    p3d_recon = [p3d_recon, x(3:5,1)];
end
figure(6);
scatter3(p3d_recon(1,:),p3d_recon(2,:),p3d_recon(3,:));

% save 3d points to text file
writematrix(p3d_recon',"p3d_recon.txt")

% For each reconstructed point, plot a line to its four nearest neighbors
%nn_ind = [];
%subject = 27;
for i=1:size(p3d_recon,2)
    dists = [];
    for j=1:size(p3d_recon,2)
        if j ~= i
            dists = [dists, [norm(p3d_recon(:,i)-p3d_recon(:,j));j]];
        end
    end
    dists_sorted = sortrows(dists.',1).';
    %nn_ind = [nn_ind, dists_sorted(2,1:3)'];
%     if i == subject
%         for k=1:5
%             ind = dists_sorted(2,k)';
%             line_seg = [p3d_recon(:,i),p3d_recon(:,ind)];
%             line(line_seg(1,:),line_seg(2,:),line_seg(3,:));
%         end
%     end
    for k=1:5
        ind = dists_sorted(2,k)';
        line_seg = [p3d_recon(:,i),p3d_recon(:,ind)];
        line(line_seg(1,:),line_seg(2,:),line_seg(3,:));
    end
end

% Distances between eye and mouth corners
disp("Left eye width:");
le_w = norm(p3d_recon(:,11)-p3d_recon(:,14));
disp(le_w);
disp("Right eye width:");
re_w = norm(p3d_recon(:,17)-p3d_recon(:,20));
disp(re_w);
disp("Mouth width:");
m_w = norm(p3d_recon(:,23)-p3d_recon(:,27));
disp(m_w);
