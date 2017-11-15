function [ err ] = func_inf_plane( p )
%UNTITLED3 Summary of this function goes here
%   p: plane at inf, 3x1 vector

global P_p_; % 3x4 projector projection mat
global wc_d_; % 3x3 camera dual image of absolute conic (DIAC) mat

Q_inf_d = [wc_d_     -wc_d_ * p;
           -p.'*wc_d_  p.'*wc_d_*p ];
       
wp_d_ = P_p_ * Q_inf_d * P_p_.'; % projector DIAC

wp_d_ = wp_d_ / wp_d_(3,3);

err(1) = wp_d_(1,3) - 640;
err(2) = wp_d_(2,3) - 800;
err(3) = wp_d_(1,3)*wp_d_(2,3)-wp_d_(1,2);

% err(1) = det(Q_inf_d);
% err(2) = wp_d_(1,3)*wp_d_(2,3)-wp_d_(1,2);
% err(3) = sqrt(wp_d_(1,1)- wp_d_(1,3)^2) - sqrt(wp_d_(2,2) - wp_d_(2,3)^2);

end

