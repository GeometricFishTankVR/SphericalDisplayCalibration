function [ x_proj, x_cam ] = func_readBlobFile( file_path )
%Read xml blob data
%   Return x_proj 2 by N, where N is the number of blobs
%   Return x_cam 2 by N, where N is the number of blobs

%file_path = ['..\Proj' num2str(proj_idx) 'PairBlobData.xml'];
xmlfile = xml2struct(file_path);    
 
str = xmlfile.Children(8).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
pt2 = data(:) -1;
x_proj = reshape(pt2,2,length(pt2)/2); % 2 by length(x_proj)

str = xmlfile.Children(4).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
pt1 = data(:);
x_cam = reshape(pt1,2,length(pt1)/2); % 2 by N

end

