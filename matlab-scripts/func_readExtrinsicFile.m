function [ Rmats, Tvecs, S, nproj, shape ] = func_readExtrinsicFile( dir )
% ------ read from extrinsics file-------
% return:
% - array of Rmats (3 x 3 x N_PROJ)
% - array of Tvecs (3 x 1 x N_PROJ)
% - display pose S (4 x 1)
% - display shape 
%% initialization 

file_extrin = xml2struct(dir);
nproj = (length(file_extrin.Children) - 3)/4;
if mod(nproj, 1.0) ~= 0
    error('func_readExtrinsicFile: extrinsic file is incorrect.')
end

Rmats = zeros(3,3,nproj);
Tvecs = zeros(3,1,nproj);
S = zeros(4,1);

%% read from file
for idx = 1:nproj
    idx_child = 2 + (idx - 1) * 4;
    
    str = file_extrin.Children(idx_child).Children(8).Children.Data;
    data = str2num(regexprep(str,'\r\n|\n|\r','')); 
    Rmats(:,:,idx) = reshape(data, 3,3)';
    
    str = file_extrin.Children(idx_child + 2).Children(8).Children.Data;
    data = str2num(regexprep(str,'\r\n|\n|\r',''));
    Tvecs(:,:,idx) = reshape(data, 3,1);
end

idx_S = nproj * 4 + 2;
str = file_extrin.Children(idx_S).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
S = reshape(data, 4,1);
end
