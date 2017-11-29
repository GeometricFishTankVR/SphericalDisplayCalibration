%%****************************************************************************
%
% Automatic Calibration Approach For Spherical Multi-projector System
%
% Created by Qian Zhou,
% Copyright 2017, University of British Columbia. All rights reserved.
%
% Distributed under the University of British Columbia academic use license
% (commercial use of this software is not permitted). See the accompanying
% file "License.txt"
%
%%***************************************************************************/
global DATA_DIR;

DATA_DIR = '..\data';
PROJ_RESOLUTION = [1280 800]; %% Modify the resolution based on the projector 


%% Initial Guess %%

% ------ read from camera intrinsics ------ %
[ cam_mat, cam_dist ] = func_readCameraIntrinsicFile( [DATA_DIR '\cam_calib.xml']);
fc_cam = [cam_mat(1,1) cam_mat(2,2)];
cc_cam = [cam_mat(1,3) cam_mat(2,3)];

% ------ read from blob data ------ %
N_PROJ = length(dir([DATA_DIR '\Proj*PairBlobData.xml']));
if N_PROJ <= 0
    error("Can't find blob data Proj*PairBlobData.xml");
end
x_cam = cell(1,N_PROJ);
x_proj = cell(1,N_PROJ);

for idx = 1: N_PROJ
    file_path = [DATA_DIR '\Proj' num2str(idx) 'PairBlobData.xml'];
    [ x_cam{idx},x_proj{idx} ] = func_read_blobxml( file_path );
end

% ------ compute projector intrinsic and extrinsic initial guess ------ %
K_p = zeros(3,3,N_PROJ); 
R_p = zeros(3,3,N_PROJ); 
T_p = zeros(3,1,N_PROJ); 
om_p = zeros(3,1,N_PROJ); 

fc_proj = zeros(2,1,N_PROJ);
cc_proj = zeros(2,1,N_PROJ);

for idx = 1: N_PROJ
    [K_p(:,:,idx),R_p(:,:,idx),T_p(:,:,idx)] = func_computeProjectorInitialGuess(... 
                                                x_cam{idx}, x_proj{idx}, ...
                                                cam_mat, cam_dist,...
                                                PROJ_RESOLUTION);
    if ~det(K_p(:,:,idx))
        error('Exit.');
    end
    om_p(:,:,idx) = rodrigues( R_p(:,:,idx) );
    fc_proj(:,:,idx) = [ K_p(1,1,idx) K_p(2,2,idx) ];
    cc_proj(:,:,idx) = [ K_p(1,3,idx) K_p(2,3,idx) ];
end

%% Triangulation %%

Xc = cell(1,N_PROJ);
Xp = cell(1,N_PROJ);
W = cell(1,N_PROJ);
sphCenter = zeros(3,1,N_PROJ);
sphR = zeros(1,N_PROJ);

for idx = 1: N_PROJ
    % triangulate to reconstruct 3D points
    [Xc{idx},Xp{idx}] = stereo_triangulation( x_cam{idx},x_proj{idx},...
                                            om_p(:,:,idx),T_p(:,:,idx),...
                                            fc_cam,cc_cam,cam_dist,0,...
                                            fc_proj(:,:,idx),cc_proj(:,:,idx),...
                                            zeros(5,1),0 );
   % compute reproj error
   [xc_err, xp_err] = func_reproj_err( Xc{idx}, x_cam{idx}, x_proj{idx}, ...
                                      fc_cam, cc_cam,cam_dist, ...
                                      fc_proj(:,:,idx),cc_proj(:,:,idx),zeros(5,1), ...
                                      om_p(:,:,idx),T_p(:,:,idx) );
   W{idx} = sqrt(xc_err+xp_err);
   % Compute sphere pose for each pair %
   [ sphCenter(:,:,idx), sphR(idx) ] = func_sphere_fit_WLS( Xc{idx}, W{idx},false );
end

%% Compute initial guess of the scale factors %%

scalorsToProj1 = ones(1, N_PROJ-1);
W_n = W{1};
Xc_n = Xc{1};
for idx = 2:N_PROJ
    % compute the scalor for projector_idx to projector_1
    scalorsToProj1(idx) = [ sphCenter(:,:,idx);sphR(idx) ] \ ...
                          [ sphCenter(:,:,1);sphR(1) ];
    if scalorsToProj1(idx) <= 0
        error('Invalid scalor value. Exit');
    end    
    % scale all the 3D data point
    T_p(:,:,idx) =  T_p(:,:,idx) * scalorsToProj1(idx);
    Xc{idx} = Xc{idx} * scalorsToProj1(idx);
    Xp{idx} = Xp{idx} * scalorsToProj1(idx);   
    W{idx} =  W{idx} * scalorsToProj1(idx);
    W_n = [W_n W{idx}];
    Xc_n = [Xc_n Xc{idx}];
end

[ sphere_p, r_p, residue ] = func_sphere_fit_WLS( Xc_n, diag(W_n),true);
title('Reconstructed 3D points of all projector: initial guess');
disp(['residue:' num2str(residue)]);

if residue > 0.25
    disp('Residue too large, Re-run the script');
    clear;
    main_automatic_calib;
    return;
end

%% lsqnonlin refinement %%

%%% if there is "not intersect!" warnings, this means:
%%% 1. The bounding circle is incorrect so that there are blobs not on the sphere.
%%% 2. Or the initial guess is too far away from the true value
%%% Try to re-run this script or re-calibrate the display

% p0: initial guess of all params
p0 = null(1);
% attach projector intrinsic and extrinsic params to p0
for idx = 1: N_PROJ
    p0 = [p0; 
        fc_proj(:,:,idx); cc_proj(:,:,idx); 
        om_p(:,:,idx); T_p(:,:,idx)];
end
p0 = [ p0; sphere_p; r_p;
       fc_cam'; cc_cam'; 
       [cam_dist(1:2) cam_dist(5)]'; cam_dist(3:4)';];
   
% set lower/upper bound of params wrt the initial guess
% ----- modify this param based on the accuracy of initial guess -----
ratio = 1.0; 
lb = p0 - abs(p0) * ratio;
ub = p0 + abs(p0) * ratio;

% setup lsqnonlin()
options = optimoptions('lsqnonlin','Display','iter');
options.Jacobian = 'on';
options.MaxIter = 200;
options.InitDamping = 500;

% Invoke optimizer
[p,resnorm] = lsqnonlin(@multi_proj_func_F, p0, lb, ub, options);   
% save to xml
if(isreal(p))
    saveParamsOpencvXml( p, [DATA_DIR '\new_params']);
    disp(['result saved to ' DATA_DIR '\new_params.xml'])
else
     error('params have imaginary part; result not saved; calibrate it again')
end
clear DATA_DIR

%% plot refined data %%
Xc_new = [];
for idx = 1: N_PROJ
   vec_p = [p( idx*10-9 : idx*10 ); 
            p( N_PROJ*10+1 : N_PROJ*10+13 )];
   [~, ~, ~, ~, X] = func_F(vec_p, idx);
   Xc_new = [Xc_new X];
end
figure,plot3(Xc_new(1,:),Xc_new(2,:),Xc_new(3,:),'.b')
hold on
[x,y,z] = sphere(20);
x1 = x*p(N_PROJ*10+4) + p(N_PROJ*10+1);
y1 = y*p(N_PROJ*10+4) + p(N_PROJ*10+2);
z1 = z*p(N_PROJ*10+4) + p(N_PROJ*10+3);
mesh(x1,y1,z1,'Marker','.')
title('Reconstructed 3D points of all projector: refined');