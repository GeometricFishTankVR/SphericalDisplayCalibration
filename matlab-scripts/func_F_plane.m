function [err, A, B, C] = func_F_plane(p, proj_idx)
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

f_p = p(1:2);
c_p = p(3:4);

om = p(5:7);
[R, J_dRdom] = rodrigues(om);
T = p(8:10);

P = [p(11:13);1];

f_c = p(14:15);
c_c = p(16:17);
kc = p(18:20);
pc = p(21:22);

%% prepare pixels from projector %%
global DATA_DIR
file_path = [DATA_DIR '..\Proj' num2str(proj_idx) 'PairBlobData.xml'];
[ x_proj, x_cam ] = func_readBlobFile( file_path );
N = length(x_proj);

x_proj_h = [x_proj; ones(1, N)]; % 3 by N

%% compute x_cam_est %%

% ray-plane intersection
KK_p = [f_p(1) 0      c_p(1);
        0      f_p(2) c_p(2);
        0      0      1];

C = -R'* T; % 3 by 1
V = ( KK_p * R ) \ x_proj_h; % vector: 3 by N
temp = bsxfun(@times,P,[V;zeros(1, N)]);
temp = sum(temp); % vector: 1 by N
lambda = bsxfun(@rdivide, -dot(P,[C; 1]), temp); % vector: 1 by N

temp = bsxfun(@times, V, lambda);
X = bsxfun(@plus, temp, C); % C + V*lambda: vector 3 by N

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


%% compute error %%
err = x_cam_est - x_cam;
err = err(:); % output: 2N by 1

%% compute jacobian %%
A = zeros(2*N, 10); % projector intrinsic and extrinsic (no lens distortion)
B = zeros(2*N, 3); % plane params
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
p1 = P(1);
p2 = P(2);
p3 = P(3);
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
    J_dxdX = JpdxdX(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dxdfc = Jpdxdfc(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dxdcc = Jpdxdcc(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dxdkc = Jpdxdkc(X1,X2,X3,fx_c,fy_c,cx_c,cy_c,kc1,kc2,kc3,pc1,pc2);
    J_dXdfp = JpdXdfp(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,p1,p2,p3,x1_p,x2_p);
    J_dXdcp = JpdXdcp(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,p1,p2,p3,x1_p,x2_p);
    J_dXdR = JpdXdR(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,p1,p2,p3,x1_p,x2_p);
    J_dXdT = JpdXdT(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,p1,p2,p3,x1_p,x2_p);
    J_dXdS = JpdXdS(fx_p,fy_p,cx_p,cy_p,r11,r21,r31,r12,r22,r32,r13,r23,r33,t1,t2,t3,p1,p2,p3,x1_p,x2_p);
    A((2*i-1):(2*i), :) = [J_dxdX*J_dXdfp J_dxdX*J_dXdcp J_dxdX*J_dXdR*J_dRdom J_dxdX*J_dXdT];
    B((2*i-1):(2*i), :) = [J_dxdX*J_dXdS];
    C((2*i-1):(2*i), :) = [J_dxdfc J_dxdcc J_dxdkc];
end

end
