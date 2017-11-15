function [ x_cam,x_proj ] = func_read_blobxml( filename )
% ------ read from blob data from xml file ------
% return:
% - x_cam: blob position in camera image plane (2 x N)
% - x_proj: blob position in projector image plane (2 x N)

xmlfile = xml2struct(filename);

str = xmlfile.Children(4).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
pt1 = data(:);
x_cam = reshape(pt1,2,length(pt1)/2); % 2 by length(x_cam)

str = xmlfile.Children(8).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
pt2 = data(:);
x_proj = reshape(pt2,2,length(pt2)/2); % 2 by length(x_proj)

end

