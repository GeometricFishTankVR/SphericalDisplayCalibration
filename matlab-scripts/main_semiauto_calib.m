%%****************************************************************************
%
% Semi-automatic Calibration Approach For Spherical Multi-projector System
% Non-linear refinement of initial guess from the Cpp project
%
% Created by Qian Zhou,
% Copyright 2017, University of British Columbia. All rights reserved.
%
% Distributed under the University of British Columbia academic use license
% (commercial use of this software is not permitted). See the accompanying
% file "License.txt"
%
%%***************************************************************************/

%% initial guess: load data %%

global DATA_DIR;

DATA_DIR = '..\data';

% ------ read from extrinsics file-------

[Rmats, Tvecs, S, N_PROJ] = func_readExtrinsicFile( [DATA_DIR '\extrinsics.xml']);

% ------ read from camera intrinsics ------

[ cam_mat, cam_dist ] = func_readCameraIntrinsicFile( [DATA_DIR '\cam_calib.xml']);

% ------ read from projector intrinsics  ------

proj_dist = zeros(1,5); 
proj_mats = zeros(3,3,N_PROJ);

for idx = 1: N_PROJ
    dir = [DATA_DIR  '\proj_calib' num2str(idx) '.xml'];
    [ proj_mats(:,:,idx) ] = func_readProjIntrinsicFile( dir );
end


%% convert mat to vector for parameter vector p0 in lsqnonlin()

fc_cam = [cam_mat(1,1) cam_mat(2,2)];
cc_cam = [cam_mat(1,3) cam_mat(2,3)];

fc_projs = zeros(1, 2, N_PROJ);
cc_projs = zeros(1, 2, N_PROJ);
Rvecs = zeros(3, 1, N_PROJ);

for idx = 1: N_PROJ
    fc_projs(:, :, idx) = [proj_mats(1,1,idx) proj_mats(2,2,idx)];
    cc_projs(:, :, idx) = [proj_mats(1,3,idx) proj_mats(2,3,idx)];
    Rvecs(:,:,idx) = rodrigues(Rmats(:,:,idx));
end

fclose('all');


%% lsqnonlin refinement %%

%%% if there is "not intersect!" warnings, this means:
%%% 1. The bounding circle is incorrect so that there are blobs not on the sphere.
%%% 2. Or the initial guess is too far away from the true value.
%%% Try to re-run this script or re-calibrate the display

% p0: initial guess of all params
p0 = null(1);

% attach projector intrinsic and extrinsic params to p0
for idx = 1: N_PROJ
    p0 = [p0; 
        fc_projs(:,:,idx)'; cc_projs(:,:,idx)'; 
        Rvecs(:,:,idx); Tvecs(:,:,idx)];
end

p0 = [ p0; S; 
       fc_cam'; cc_cam'; 
       [cam_dist(1:2) cam_dist(5)]'; cam_dist(3:4)';];

% set lower/upper bound of params wrt the initial guess
ratio = .5; %% play with this param based on the accuracy of initial guess
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
    close('all');
else
     error('params have imaginary part; result not saved; calibrate it again')
end
 
clear DATA_DIR