function [err, A, B, C, X] = func_F(p, proj_idx)
% p = [ fx_p, fy_p, cx_p, cy_p, //1-4
%       om1, om2, om3, //5-7
%       t1, t2, t3, //8-10
%       a, b, c, r, //11-14
%       fx_c, fy_c, cx_c, cy_c, k1_c, k2_c, k3_c, p1_c, p2_c] //15-23
% p has to be a column vector
% Return: 
%       A : NumberOfBlobs x 10 ->projector intrinsic and extrinsic
%       B : NumberOfBlobs x 4 ->sphere pose
%       C : NumberOfBlobs x 9 ->camera intrinsics (with lens distortions)
%       X : 3 x NumberOfBlobs ->3d pts
f_p = p(1:2);
c_p = p(3:4);

om = p(5:7);
[R, J_dRdom] = rodrigues(om);
T = p(8:10);

S = p(11:13);
r = p(14);

f_c = p(15:16);
c_c = p(17:18);
kc = p(19:21);
pc = p(22:23);

%% prepare pixels from projector %%
global DATA_DIR;
file_path = [DATA_DIR '\Proj' num2str(proj_idx) 'PairBlobData.xml'];
xmlfile = xml2struct(file_path);    
    
str = xmlfile.Children(8).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
pt2 = data(:) -1;
x_proj = reshape(pt2,2,length(pt2)/2); % 2 by length(x_proj)
N = length(x_proj);
 
x_proj_h = [x_proj; ones(1, N)]; % 3 by N

%% compute x_cam_est %%

% ray-sphere intersection
KK_p = [f_p(1) 0      c_p(1);
        0      f_p(2) c_p(2);
        0      0      1];

C = -R'* T; % 3 by 1
C_S = C - S; % 3 by 1


V = ( KK_p * R ) \ x_proj_h; % vector: 3 by N

a = V(1,:).^2 +  V(2,:).^2 + V(3,:).^2; % row vector: 1 by N
b = 2 * C_S' * V; % row vector: 1 by N
c = norm(C_S)^2 - r^2; % row scalar: 1 by 1

b2_4ac = b.^2 - 4*c*a; % vector: 1 by N

lambda = (sqrt(b2_4ac) - b)./a/2; % vector: 1 by N
temp = bsxfun(@times, V, lambda);
X = bsxfun(@plus, temp, C); % C + V*lambda: vector 3 by N

outLiner_idx = find(b2_4ac < 0);
if ~isempty( outLiner_idx )
    disp(['not intersect!'])
    lambda(outLiner_idx) = - transpose(V(:,outLiner_idx).' * C_S) ./ a(outLiner_idx);
    temp = bsxfun(@times, V(:,outLiner_idx), lambda(outLiner_idx));
    X(:,outLiner_idx) = bsxfun(@plus, temp, C); % C + V*lambda: vector 3 by N
end



% camera projection

x1 = X(1,:) ./ X(3,:); % vector: 1 by N
y1 = X(2,:) ./ X(3,:); % vector: 1 by N
r2 = x1.^2 + y1.^2; % vector: 1 by N
r4 = r2.^2; % vector: 1 by N
r6 = r4.*r2; % vector: 1 by N
x2 = x1.*(ones(1, N) + kc(1)*r2 + kc(2)*r4 + kc(3)*r6)...
     + 2*pc(1)*x1.*y1 + pc(2)*(r2 + 2*x1.^2); % vector: 1 by N
y2 = y1.*(ones(1, N) + kc(1)*r2 + kc(2)*r4 + kc(3)*r6)...
     + 2*pc(2)*x1.*y1 + pc(1)*(r2 + 2*y1.^2); % vector: 1 by N
xp = f_c(1)*x2 + c_c(1)*ones(1,N); % vector: 1 by N
yp = f_c(2)*y2 + c_c(2)*ones(1,N); % vector: 1 by N
x_cam_est = [xp; yp]; % 2 by N

%% camera observations x_cam %%

str = xmlfile.Children(4).Children(8).Children.Data;
data = str2num(regexprep(str,'\r\n|\n|\r',''));
pt1 = data(:);
x_cam = reshape(pt1,2,length(pt1)/2); % 2 by N

%% compute error %%
err = x_cam_est - x_cam;
err = err(:); % output: 2N by 1

%% compute jacobian %%
A = zeros(2*N, 10); % projector intrinsic and extrinsic
B = zeros(2*N, 4); % sphere pose
C = zeros(2*N, 9); % camera intrinsics (with lens distortions)

fx_p = f_p(1);
fy_p = f_p(2);
cx_p = c_p(1);
cy_p = c_p(2);
r11 = R(1,1);
r12 = R(1,2); 
r13 = R(1,3);
r21 = R(2,1);
r22 = R(2,2);
r23 = R(2,3);
r31 = R(3,1);
r32 = R(3,2);
r33 = R(3,3);
t1 = T(1);
t2 = T(2);
t3 = T(3);
s1 = S(1);
s2 = S(2);
s3 = S(3);
fx_c = f_c(1);
fy_c = f_c(2);
cx_c = c_c(1);
cy_c = c_c(2);
kc1 = kc(1);
kc2 = kc(2);
kc3 = kc(3);
pc1 = pc(1);
pc2 = pc(2);

for i = 1:N
    X1 = X(1, i);
    X2 = X(2, i);
    X3 = X(3, i);
    x1_p = x_proj(1, i);
    x2_p = x_proj(2, i);
    J_dxdX = JdxdX(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dxdfc = Jdxdfc(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dxdcc = Jdxdcc(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dxdkc = Jdxdkc(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dXdfp = JdXdfp(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,s1,s2,s3,r,x1_p,x2_p);
    J_dXdcp = JdXdcp(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,s1,s2,s3,r,x1_p,x2_p);
    J_dXdR = JdXdR(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,s1,s2,s3,r,x1_p,x2_p);
    J_dXdT = JdXdT(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,s1,s2,s3,r,x1_p,x2_p);
    J_dXdS = JdXdS(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,s1,s2,s3,r,x1_p,x2_p);
    A((2*i-1):(2*i), :) = [J_dxdX*J_dXdfp J_dxdX*J_dXdcp J_dxdX*J_dXdR*J_dRdom J_dxdX*J_dXdT];
    B((2*i-1):(2*i), :) = [J_dxdX*J_dXdS];
    C((2*i-1):(2*i), :) = [J_dxdfc J_dxdcc J_dxdkc];
end

end
