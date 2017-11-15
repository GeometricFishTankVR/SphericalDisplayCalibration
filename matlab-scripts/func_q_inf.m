function [ err ] = func_q_inf( q )
%UNTITLED3 Summary of this function goes here
%   q: absolute dual quadirc, 4x1 vector

global P_p_; % 3x4 projector projection mat
global wc_d_; % 3x3 camera dual image of absolute conic (DIAC) mat

Q_inf_d = [wc_d_ q(1:3);
           q(1:3)' q(4)];

wp_d_ = P_p_ * Q_inf_d * P_p_.'; % projector DIAC

wp_d_ = wp_d_ / wp_d_(3,3);

err(1) = wp_d_(1,3) - 640;
err(2) = wp_d_(2,3) - 800;
err(3) = wp_d_(1,2)  - 640*800;
err(4) = q(4) - q(1:3)' * wc_d_ * q(1:3);
end

