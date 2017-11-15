function [ proj_mat ] = func_readProjIntrinsicFile( dir )
% ------ read from projector intrinsics  ------
% return:
% - projector intrinsic mat proj_mat(3 x 3)
% ignore lens distortions of projectors
    file_intrin = xml2struct(dir);
    str = file_intrin.Children(14).Children(8).Children.Data;
    data = str2num(regexprep(str,'\r\n|\n|\r',''));
    proj_mat = reshape(data, 3,3)';
end

