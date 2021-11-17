clear all; close all;

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

% Calculate relative orientation of two views
R_prime = R_l*transpose(R_r);
T_prime = T_l-R_prime*T_r;
%     R_prime = transpose(R_r)*R_l;
%     T_prime = transpose(R_r)*T_l-transpose(R_r)*T_r;
disp("Relative R");
disp(R_prime);
disp("Relative T");
disp(T_prime);

S=[0 -T_prime(3,1) T_prime(2,1);
   T_prime(3,1) 0 -T_prime(1,1);
   -T_prime(2,1) T_prime(1,1) 0];
E = transpose(S)*R_prime;
disp('E:');
disp(E);
F = inv(transpose(W_l))*E*inv(W_r);
disp('F:');
disp(F);
    
% For each of the left features, draw epipolar line on right image
% Real image is 2000 x 1600
% Compressed image is 577 x 434
% compression_scale = [2048/577; 1536/434];
p2d_lface = ParseMat("resources/pts_left.txt", 2, 0);
p2d_rface = ParseMat("resources/pts_right.txt", 2, 0);
    
% figure(4);
% scatter(p2d_rface(1,:),-1*p2d_rface(2,:));
% hold on;
% axis equal
% for i=1:size(p2d_lface,2)
%     coeffs = transpose(F)*[p2d_lface(:,i); 1];
%     syms c_r
%     r_r = (-coeffs(3,1)-coeffs(1,1)*c_r)/coeffs(2,1);
%     % plot epipolar line
%     fplot(-1*r_r,[0 2048]);
% end
% xlim([0, 2048]);
% ylim([-1536, 0]);

% Rectification
base_l = T_l - T_r;
rp_l = rectify_r(R_l, base_l);
rp_r = rectify_r(R_r, base_l);
% Rotate along x to align axes
% find angle between z axes
zl = rp_l*R_l*[0;0;1];
zr = rp_r*R_r*[0;0;1];
theta_z = acos(dot(zl,zr));
rz_l = axang2rotm([(rp_l*R_l*[1;0;0])', -theta_z/2]);
rz_r = axang2rotm([(rp_r*R_r*[1;0;0])', theta_z/2]);

R_lrect = rz_l*rp_l*R_l;
%R_lrect = rp_l*R_l;
R_rrect = R_lrect*R_prime;

% plot coordinate systems
% original
figure(1);
bplot = [T_l,T_r];
plot3(bplot(1,:),bplot(2,:),bplot(3,:),'cyan');
hold on;
% plot_coord(R_l,T_l,[100,100,100]);
% plot_coord(R_r,T_r,[100,100,100]);
% plot_coord(rp_l*R_l,T_l,[100,100,100]);
% plot_coord(rp_r*R_r,T_r,[100,100,100]);
% plot_coord(R_lrect,T_l,[100,100,100]);
% plot_coord(R_rrect,T_r,[100,100,100]);
% xlim([-400, 400]);
% ylim([-400, 400]);
% zlim([-400, 400]);

% Scale down focal length
scale = 0.7;
W_l(1,1) = scale*W_l(1,1);
W_l(2,2) = scale*W_l(2,2);
W_r(1,1) = scale*W_r(1,1);
W_r(2,2) = scale*W_r(2,2);

% Transform points to rectified coordinate frame
p2d_lprime = zeros(3,size(p2d_lface,2));
p2d_rprime = zeros(3,size(p2d_rface,2));
for i=1:size(p2d_lface,2)
    p2d_lprime(:,i) = W_l*R_lrect*inv(W_l)*[p2d_lface(:,i);1];
    p2d_rprime(:,i) = W_r*R_rrect*inv(W_r)*[p2d_rface(:,i);1];
end
p2d_lprime = p2d_lprime(1:2,:)./p2d_lprime(3,:);
p2d_rprime = p2d_rprime(1:2,:)./p2d_rprime(3,:);

% recompute fundamental matrix
% Calculate relative orientation of two views
disp(rz_l*rp_l*(T_l-R_prime*T_r));

R_prime = R_lrect*transpose(R_rrect);
T_prime = T_l-R_prime*T_r;

S=[0 -T_prime(3,1) T_prime(2,1);
   T_prime(3,1) 0 -T_prime(1,1);
   -T_prime(2,1) T_prime(1,1) 0];
E = transpose(S)*R_prime;
disp('E:');
disp(E);
F = inv(transpose(W_l))*E*inv(W_r);
disp('F:');
disp(F);

face_corners = [11, 14, 17, 20, 23, 27];
figure(4);
hold on;
scatter(p2d_rprime(1,:),-1*p2d_rprime(2,:));
axis equal
for i=1:size(p2d_lprime,2)
    if ismember(i, face_corners)
        coeffs = transpose(F)*[p2d_lprime(:,i); 1];
        syms c_r
        r_r = (-coeffs(3,1)-coeffs(1,1)*c_r)/coeffs(2,1);
        % plot epipolar line
        fplot(-1*r_r,[0 2000]);
    end
end
xlim([0, 2000]);
ylim([-1536, 0]);
imdir = R_rrect*[1;0;0];
lnorm = R_rrect*[0;0;1];
myIm = imread('faceimage/right_face.jpg');
imsurf(myIm,[0,0,0],lnorm',imdir',[]);
%imsurf(myIm,[0,0,0],[0,0,1],[1,0,0],[]);

%imsurf(imageIn,upperLeftPoint3,normal,imXDirVec,scale,varargin)
figure(10);
hold on;
scatter(p2d_lprime(1,:),-1*p2d_lprime(2,:));
imdir = R_lrect*[1;0;0];
lnorm = R_lrect*[0;0;1];
disp(dot(imdir,lnorm));
myIm = imread('faceimage/left_face.jpg');
%imsurf(myIm,[0,-1536,0],lnorm',imdir',[]);
imsurf(myIm,[0,0,0],[0,0,1],[1,0,0],[]);
%xlim([0, 2000]);
%ylim([-1536, 0]);