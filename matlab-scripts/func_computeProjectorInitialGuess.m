function [ K_p,R_p,T_p  ] = func_computeProjectorInitialGuess( x_cam, x_proj, ...
                                                  K_cam, dist_cam,...
                                                  resol_proj)
% Compute Initial guess of projector intrinsics and Extrinsic
% Assumption:
% - Known principle point roughly at the bottom center of projector 
%   image plane (if projector resolution is m x n, then principle point 
%   is set at m/2 x n).
% - No strong lens distortion of projector 
% Input:
% - x_cam: blob position in camera image plane (2 x N)
% - x_proj: blob position in projector image plane (2 x N)
% - K_cam: camera intrinsic mat (3 x 3)
% - dist_cam: camera lens distortion mat (1 x 5)
% - resol_proj: projector image plane resolution (1 x 2) [width, height]
% return:
% - Projector intrinsic Matrix K_p(3 x 3)
% - Projector extrinsic rotation R_p(3 x 3)
% - Projector extrinsic translation T_p (3 x 1)

%% setup %%
fc_cam = [K_cam(1,1) K_cam(2,2)];
cc_cam = [K_cam(1,3) K_cam(2,3)];

proj_cc = [resol_proj(1)/2 resol_proj(2)];
% ----- undistort camera points -----
x_cam_nml = normalize_pixel(x_cam,fc_cam,cc_cam,dist_cam,0);
new_x_cam = K_cam * [x_cam_nml;ones(1,length(x_cam_nml))];
x_cam = new_x_cam(1:2,:);

%% projective reconstruction %%
%[F, ~, ~] = estimateFundamentalMatrix(x_cam', x_proj');
F = estimateFundamentalMatrixIter(x_cam,x_proj);

%figure, plot(x_cam(1,:), x_cam(2,:),'r.')
%hold on
%plot(x_proj(1,:), x_proj(2,:),'b.')
%hold off

% find the epipole in second view
[U,S,V] = svd(F'); 

ep = V(:,3);
Ep_ss = skew_sym(ep);
     
A = Ep_ss * F;
a = ep;

P_p_ = [A a];
wc_d_ = K_cam * K_cam'; % camera DIAC

%% find homography H %%

% find plane at inf
syms q1 q2 q3

wc_ = inv(wc_d_);

q4 = [q1 q2 q3] * wc_ * [q1 q2 q3].';
q = [q1;q2;q3;q4];

Q_inf_d = [wc_d_     q(1:3);
           q(1:3).'  q(4) ];
       
wp_d_ = P_p_ * Q_inf_d * P_p_.'; % projector DIAC

temp = solve( wp_d_(1,3) - proj_cc(1) * wp_d_(3,3)==0,  ...
              wp_d_(2,3) - proj_cc(2) * wp_d_(3,3)==0 , ...
              wp_d_(1,2) - proj_cc(1)* proj_cc(2) * wp_d_(3,3)== 0);

clear q q1 q2 q3 q4 wp_d_ Q_inf_d

q1 = double(temp.q1);
q2 = double(temp.q2);
q3 = double(temp.q3);

% global PA Pa wc_d;
% PA = A;
% Pa = a;
% wc_d = wc_d_;
% 
% options = optimoptions('lsqnonlin','Algorithm','levenberg-marquardt','Display','iter');
% 
% options.Jacobian = 'off';
% options.MaxIter = 200;
% 
%  [inf_p,~] = lsqnonlin(@func_computeInfPlane, [0;0;0], [], [], options); 
%  [~,wp_d] = func_computeInfPlane(inf_p);
%  wp_d = wp_d / wp_d(3,3)
%  fcp_est = [sqrt(wp_d(1,1) - wp_d(1,3)^2) sqrt(wp_d(2,2) - wp_d(2,3)^2)]
%  ccp_est = [ wp_d(1,3) wp_d(2,3)]
%  K_p_est = [fcp_est(1) 0 ccp_est(1); 0 fcp_est(2) ccp_est(2); 0 0 1];
%  norm(K_p_est * K_p_est' - wp_d)
 %% find intrinsic K_p %% 
K_p_est = zeros(3);
detQ = 1;
for i = 1 : length(q1)
    q4 = [q1(i) q2(i) q3(i)] * wc_ * [q1(i) q2(i) q3(i)].';
    Q_inf_d = [ wc_d_     [q1(i);q2(i);q3(i)];
               [q1(i) q2(i) q3(i)]  q4 ];
    [u,s,v] = svd(Q_inf_d); 
    s(4,4) = 0;
    Q_inf_d = u * s * v'; 
    wp_d = P_p_ * Q_inf_d * P_p_'; % projector DIAC
    wp_d = wp_d / wp_d(3,3);
    
    fcp_est = [sqrt(wp_d(1,1) - wp_d(1,3)^2) sqrt(wp_d(2,2) - wp_d(2,3)^2)];
    ccp_est = [ wp_d(1,3) wp_d(2,3)];
    if (isreal(fcp_est)  && abs(det(Q_inf_d)) < abs(detQ))
        fcp = fcp_est;
        ccp = ccp_est;
        K_p_est = [fcp(1) 0 ccp(1); 0 fcp(2) ccp(2); 0 0 1]; 
        detQ = det(Q_inf_d);
        err = norm(K_p_est * K_p_est' - wp_d)
    end
end

%% find extrinsic R_p T_p %%
%err = norm(K_p_est * K_p_est' - wp_d)

% ----- setup params to find extrinsics ----- %
% modify these two params if the reproj error is large or time is too long
MAX_ITER = 500; 
S_THRES = 0.99; % ideally S(2,2) should be close to 1

% ----- find extrinsics ----- %
if det(K_p_est)
    K_p = K_p_est; 
    x_cam_nml = normalize_pixel(x_cam,fc_cam,cc_cam,zeros(5,1),0);
    x_proj_nml = normalize_pixel(x_proj,fcp,ccp,zeros(5,1),0);
    iter = 1;
   while iter < MAX_ITER
        %F = estimateFundamentalMatrix(x_cam_nml',x_proj_nml');
        F = estimateFundamentalMatrixIter(x_cam_nml,x_proj_nml);
        [~,S,~] = svd(F);
        sratio = S(2,2)/S(1,1)
        if(sratio > S_THRES)
            break
        end
        iter = iter + 1;
   end
   if iter >=  MAX_ITER
       error('Time out. Cannot find correct Fundamental Matrix F. Fail to compute extrinsics. ');
   end
    [ R_p, T_p ] = func_compute_extrin( x_cam, x_proj, F, ...
                                        fc_cam,cc_cam,zeros(5,1),...
                                        fcp_est,ccp_est,zeros(5,1) );
else
    error('Invalid projector intrinsic solution; Result not saved.')
end

end

