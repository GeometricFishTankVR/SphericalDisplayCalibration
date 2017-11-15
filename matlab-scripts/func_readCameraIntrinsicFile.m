function [ cam_mat, cam_dist ] = func_readCameraIntrinsicFile( dir )
% ------ read from camera intrinsics ------
% return:
% - camera intrinsic mat (3 x 3)
% - camera lens distortion params (1 x 5)

file_intrin = xml2struct(dir);

str = file_intrin.Children(14).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
cam_mat = reshape(data, 3,3)';

str = file_intrin.Children(16).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
cam_dist = reshape(data, 1,5);

end

