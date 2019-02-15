function [ err,F ] = func_fundemental_to_err( f12 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global xcam xproj; 

t = f12(1:3);
m = f12(4:12);
M = reshape(m,3,3);
t_ss= skew_sym(t);
F = t_ss * M;
err = 0;
for i = 1: length(xcam)
    erri = xproj(:,i)' * F * xcam(:,i);
    Fxcam = F * xcam(:,i);
    Ftxp = F' * xproj(:,i);
    erri = erri / ( Fxcam(1)^2 + Fxcam(2)^2 + Ftxp(1)^2 + Ftxp(2)^2);
    err = err + erri;
end

end

