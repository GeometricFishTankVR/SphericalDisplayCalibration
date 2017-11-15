function [ reproj_error1, reproj_error2 ] = func_reproj_err( C_X_cam1, p1, p2, fc1, cc1,dist1,fc2,cc2,dist2,om,T )
% Compute reprojection error from a stereo pair
% Input:
% - C_X_cam1: 3d pts in the coordinate of the first camera
% - p1: 2d pts in the image plane of the first camera
% - p2: 2d pts in the image plane of the second camera
% - fc1, fc2: focal length of the 1st and 2nd cameras
% - cc1, cc2: principle point of the 1st and 2nd cameras
% - dist1, dist2: lens distortion of the 1st and 2nd cameras
% - om,T: rotation vector and translation vector between 1st and 2nd cameras
% Output:
% - reproj_error1, reproj_error2: reprojection error of 1st and 2nd cameras

x_reproj1= project_points2(C_X_cam1,[0;0;0],[0;0;0],fc1,cc1,dist1,0);
x_reproj2= project_points2(C_X_cam1,om,T,fc2,cc2,dist2,0);

reproj_vector1 = p1 - x_reproj1;
reproj_vector2 = p2 - x_reproj2;

reproj_error1 = reproj_vector1(1,:).^2 + reproj_vector1(2,:).^2;
reproj_error2 = reproj_vector2(1,:).^2 + reproj_vector2(2,:).^2;

reproj_error_pixel_s = sum(reproj_error1 + reproj_error2)/2/length(reproj_error1);

disp(['sum of reprojection error in two images'])
reproj_err = sqrt(reproj_error_pixel_s);
disp(reproj_err);

% %% plot reprojection error %%
% figure,
% scatter(p1(1,:), p1(2,:))
% hold on
% scatter(x_reproj1(1,:), x_reproj1(2,:))
% hold off
% 
% figure,
% scatter(p2(1,:), p2(2,:),'b')
% hold on
% scatter(x_reproj2(1,:), x_reproj2(2,:),'r')
% hold off

end

