function [ err, w_p_d] = func_computeInfPlane(p)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
global PA;
global Pa;
global wc_d;

w_p_d = (PA - Pa * p.') * wc_d * (PA - Pa * p.').';

err(1) = w_p_d(1,3) - 512 * w_p_d(3,3);
err(2) = w_p_d(2,3) - 768 * w_p_d(3,3);
err(3) = w_p_d(1,2) - 512 * 768 * w_p_d(3,3);

%err = abs(err1) + err2^2 + err3^3;
end

