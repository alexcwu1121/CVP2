function [P_init, error] = MultiCalib(p3d, p2d, tol, W_supply)
    while true
        P_init = CalibrateCamera(p3d, p2d, 3, 3, true, false, W_supply);
        
        if size(W_supply) ~= 0
            P_init = W_supply*P_init;
        end
        
%         x0 = [transpose(P_init(1,1:4));
%             transpose(P_init(2,1:4));
%             transpose(P_init(3,1:4))];
        error = ReprojError(P_init, p3d, p2d, false);
        if error < tol
            break;
        end
    end
end