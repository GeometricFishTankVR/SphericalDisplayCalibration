function [ err,f ] = func_epipole_to_err( e )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
    global A;
    
    ex = [0 -e(3) e(2) ; e(3) 0 -e(1) ; -e(2) e(1) 0 ];
    z33 = zeros(3,3);
    E = [ex z33 z33; z33 ex z33; z33 z33 ex];
    [U S V] = svd(E);
    Ur = U(:,diag(S) > 0.00000001);
    [U S V] = svd(A * Ur);
    xr = V(:,6);
    f = Ur * xr;

    err = A * f;

%     F = reshape(f,3,3)';
%     err = zeros(length(xcam),1);
%     for i = 1: length(xcam)
%         erri = xproj(:,i)' * F * xcam(:,i);
%         Fxcam = F * xcam(:,i);
%         Ftxp = F' * xproj(:,i);
%         erri = erri / ( Fxcam(1)^2 + Fxcam(2)^2 + Ftxp(1)^2 + Ftxp(2)^2);
%         err(i)  = erri;
%     end
    
end

