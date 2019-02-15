function [ F  ] = estimateFundamentalMatrixIter(x_cam,x_proj)

[F0, ~, ~] = estimateFundamentalMatrix(x_cam', x_proj');

x_proj = [x_proj; ones(1, length(x_proj))];
x_cam = [x_cam; ones(1, length(x_cam))];

x1x2 = x_cam(1,:) .*  x_proj(1,:);
x1y2 = x_cam(1,:) .*  x_proj(2,:);
x2y1 = x_cam(2,:) .*  x_proj(1,:);
y1y2 = x_cam(2,:) .*  x_proj(2,:);

global A xcam xproj;
A = [x1x2' x2y1' x_proj(1,:)' x1y2' y1y2' x_proj(2,:)' x_cam(1,:)' x_cam(2,:)' ones(length(x_proj),1)];
xcam = x_cam;
xproj = x_proj;

[U, S, V] = svd(F0);

e0 = V(:,3);

f0 = F0';
f0 = f0(:);

options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');

options.Jacobian = 'off';
options.MaxIter = 200;

[e,~] = lsqnonlin(@func_epipole_to_err, e0, [], [], options); 

[~, fn] = func_epipole_to_err(e);

F = reshape(fn,3,3)';

%% method2
% [F0, ~, ~] = estimateFundamentalMatrix(x_cam', x_proj');
% 
% [~, ~, V] = svd(F0');
% e0 = V(:,3);
% 
% Ep_ss = skew_sym(e0);
%      
% M0 = Ep_ss * F0;
% t0 = e0;
% 
% f12_0 = [t0(:);M0(:)];
% 
% 
% global xcam xproj;
% xproj = [x_proj; ones(1, length(x_proj))];
% xcam = [x_cam; ones(1, length(x_cam))];
%  
% options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
% options.Jacobian = 'off';
% options.MaxIter = 200;
% 
% [f12,~] = lsqnonlin(@func_fundemental_to_err, f12_0, [], [], options); 
% [~,F] = func_fundemental_to_err(f12);
% rankF = rank(F)
% normf = norm(F(:))

end

