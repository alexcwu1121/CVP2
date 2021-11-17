function [] = double_driver(p3d_file, p2d_left, p2d_right)
    % double_driver("resources/pts_3D.txt","resources/pts_2D_left.txt", "resources/pts_2D_right.txt")
    close all;
    p3d = ParseMat(p3d_file, 2, 0);
    p2d_l = ParseMat(p2d_left, 2, 0);
    p2d_r = ParseMat(p2d_right, 2, 0);
    
    % uses figures 1-3
    plot_points(p3d, p2d_l, p2d_r);
    
    % First solve the left projection matrix and derive intrinsic
    % parameters
%     W_set = 0.7*[2.2260, 0, 0.7677;
%             0, 2.3125, 0.3073;
%             0, 0, 0.0010]*10^3;
%     W_set = [4.5, 0, -1.1240;
%             0, 4.5, 0.6433;
%             0, 0, 0.0010]*10^3;
%     [P_init_l, error_l] = MultiCalib(p3d, p2d_l, 1*10^6, []);
%     [P_init_r, error_r] = MultiCalib(p3d, p2d_r, 1*10^6, []);
    
    [P_init_l, error_l] = MultiCalib(p3d, p2d_l, 1*10^6, []);
    [W_init_l, R_init_l, T_init_l] = recover_params(P_init_l);
    [P_init_r, error_r] = MultiCalib(p3d, p2d_r, 1*10^6, W_init_l);
    
%     [P_init_l, error_l] = MultiCalib(p3d, p2d_l, 5*10^5, W_set);
%     [P_init_r, error_r] = MultiCalib(p3d, p2d_r, 5*10^5, W_set);
    
    [W_init_l, R_init_l, T_init_l] = recover_params(P_init_l);
    [W_init_r, R_init_r, T_init_r] = recover_params(P_init_r);
    
    disp("Initial left projection matrix:");
    disp(P_init_l);
    disp("Average left reprojection error pre-optimization:");
    disp(error_l/size(p3d,2));
    disp("Initial right projection matrix:");
    disp(P_init_r);
    disp("Average right reprojection error pre-optimization:");
    disp(error_r/size(p3d,2));
    
    % Combine left and right extrinsic camera parameters
    % These aren't actuall rotation vectors, just flattened rot matrices
    rvec_l = R_init_l(:);
    rvec_r = R_init_r(:);
    wvec_l = [W_init_l(1,1);W_init_l(2,2);W_init_l(1,3);W_init_l(2,3)];
    wvec_r = [W_init_r(1,1);W_init_r(2,2);W_init_r(1,3);W_init_r(2,3)];
    x0 = [wvec_l;rvec_l;T_init_l;wvec_r;rvec_r;T_init_r];
    p2d = {p2d_l,p2d_r};
    
    % Plot linear solutions to P
    figure(2);hold on;
    p2rep_l = [];
    for i=1:size(p3d,2)
        p2rep_l = [p2rep_l,(1/P_init_l(3,4))*P_init_l*[p3d(:,i);1]];
    end
    scatter(p2rep_l(1,:),p2rep_l(2,:),'x','green');
    figure(3);hold on;
    p2rep_l = [];
    for i=1:size(p3d,2)
        p2rep_l = [p2rep_l,(1/P_init_r(3,4))*P_init_r*[p3d(:,i);1]];
    end
    scatter(p2rep_l(1,:),p2rep_l(2,:),'x','green');
    
    % Optimize projection matrices 'Display','iter'
    options = optimoptions('fmincon','Algorithm','interior-point',...
                    'MaxFunctionEvaluations', 10*10^5,...
                    'MaxIterations',1*10^4,...
                    'StepTolerance', 1e-10,...
                    'Display','iter');
    x = fmincon(@(x)combined_error(x, p3d, p2d),x0,...
        [],[],[],[],[],[],...
        @(x)OrthCons(x, true),options);

    W_l = [x(1,1), 0, x(3,1);
        0, x(2,1), x(4,1);
        0, 0, 1];
    W_r = [x(17,1), 0, x(19,1);
        0, x(18,1), x(20,1);
        0, 0, 1];
    R_l = reshape(x(5:13,1),[3,3]);
    R_r = reshape(x(21:29,1),[3,3]);
    T_l = x(14:16,1);
    T_r = x(30:32,1);
    P_l = W_l*[R_l, T_l];
    P_r = W_r*[R_r, T_r];

    disp("Left Error");disp(ReprojError(P_l, p3d, p2d_l, false));
    disp("R_l:");disp(R_l);
    disp("T_l:");disp(T_l);
    disp("W_l:");disp(W_l);
    
    disp("Right Error");disp(ReprojError(P_r, p3d, p2d_r, false));
    disp("R_r:");disp(R_r);
    disp("T_r:");disp(T_r);
    disp("W_r:");disp(W_r);
    
    % Plot nonlinear projection matrices
    figure(2);
    hold on;
    p2rep_l = [];
    for i=1:size(p3d,2)
        p2rep_l = [p2rep_l,(1/P_l(3,4))*P_l*[p3d(:,i);1]];
    end
    scatter(p2rep_l(1,:),p2rep_l(2,:),'x','red');
    legend('GTruth', 'Lin', 'Lin+NonLin');
    
    figure(3);
    hold on;
    p2rep_r = [];
    for i=1:size(p3d,2)
        p2rep_r = [p2rep_r,(1/P_r(3,4))*P_r*[p3d(:,i);1]];
    end
    scatter(p2rep_r(1,:),p2rep_r(2,:),'x','red');
    legend('GTruth', 'Lin', 'Lin+NonLin');
    pause(0.1)
    
    disp(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    
    % Calculate relative orientation of two views
    R_prime = R_l*transpose(R_r);
    T_prime = T_l-R_prime*T_r;
    %T_prime = T_l-T_r;
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
    %p2d_lface = ParseMat("resources/pts_2D_left.txt", 2, 0);
    p2d_rface = ParseMat("resources/pts_right.txt", 2, 0);
    %p2d_rface = ParseMat("resources/pts_2D_right.txt", 2, 0);
    disp(p2d_lface);
    disp(p2d_rface);
    
    % 11, 14, 17, 20, 23, 27
    face_corners = [11, 14, 17, 20, 23, 27];
    figure(4);
    %indices = randi([1 size(p2d_lface,2)],1,10);
    indices = 1:size(p2d_lface,2);
    scatter(p2d_rface(1,indices),-1*p2d_rface(2,indices));
    hold on;
    axis equal
    for i=indices
        if ismember(i, face_corners)
            coeffs = transpose(F)*[p2d_lface(:,i); 1];
            syms c_r
            r_r = (-coeffs(3,1)-coeffs(1,1)*c_r)/coeffs(2,1);
            % plot epipolar line
            fplot(-1*r_r,[0 2048]);
        end
    end
    xlim([0, 2048]);
    ylim([-1536, 0]);
end