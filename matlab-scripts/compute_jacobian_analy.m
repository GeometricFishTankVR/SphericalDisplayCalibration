% Compute Jacobian of func_F() wrt params p = : 

% variables: 
%             fx_p, fy_p - projector focal length
%             cx_p, cy_p - projector principle point
%             r11, r21 ..., r33 - projector rotation in matrix
%             t1, t2, t3 - projector translation
%             s1,s2,s3,r - sphere pose
%             fx_c, fy_c - camera focal length
%             cx_c, cy_c - camera principle point
%             kc1, kc2, kc3, pc1, pc2 - camera lens distortions
%             (dbg: ignored for now) projector lens distortions

% input: x_p = [x1_p, x2_p] 2D image point of projector
% output: x_c = [x1_c, x2_c] 2D image point of camera

% intermediate variables: lambda - depth in Projector Coordinate 
%                         C      - projector center of projection
%                         V      - Ray in Projector Coordinate

% Useful functions to convert analy Jacobian result 
% to functions in different context: 
% matlabFunc(); latex(); ccode();

% variables: 
syms fx_p fy_p cx_p cy_p;
syms r11 r21 r31 r12 r22 r32 r13 r23 r33;
syms t1 t2 t3;
syms s1 s2 s3 r;
syms fx_c fy_c cx_c cy_c;
syms kc1 kc2 kc3 pc1 pc2;

% input & outputs
syms x1_p x2_p x1_c x2_c;

% intermediate variables
syms KK_p R T S X;
syms lambda C C_S V;
syms a b c;

%% ray-sphere intersection %%

KK_p = [fx_p 0      cx_p;
        0     fy_p  cy_p;
        0      0      1];

R = [ r11 r12 r13;...
      r21 r22 r23;...
      r31 r32 r33; ];    
T = [t1;t2;t3];

S = [s1;s2;s3];

C = -R\T; % 3 by 1
C_S = C - S; % 3 by 1

V = ( KK_p * R ) \ [x1_p;x2_p;1]; % 3 by 1

a = dot(conj(V),V); % scalar
b = 2*dot(conj(V), C_S); % scalar
c = dot(conj(C_S), C_S) - r^2; % scalar

b2_4ac = b^2 - 4*a*c; 
lambda = (sqrt(b2_4ac) - b)/a/2; 
X = C + V * lambda;

%% camera projection %%
 syms X1 X2 X3;
x1 = X1 / X3; % scalar
y1 = X2 / X3; % scalar
% x1 = X(1) / X(3); % scalar
% y1 = X(2) / X(3); % scalar
r2 = x1^2 + y1^2; 
r4 = r2^2; 
r6 = r4*r2; 
x2 = x1*(1 + kc1*r2 + kc2*r4 + kc3*r6) + 2*pc1*x1*y1 + pc2*(r2 + 2*x1^2);
y2 = y1*(1 + kc1*r2 + kc2*r4 + kc3*r6) + 2*pc2*x1*y1 + pc1*(r2 + 2*y1^2); 
x1_c = fx_c * x2 + cx_c; 
x2_c = fy_c * y2 + cy_c; 
x_c = [x1_c;x2_c];

%% jacobians %%

% ray-sphere intersection

JdXdfp = jacobian(X, [fx_p fy_p]); % 3 by 2
JdXdcp = jacobian(X, [cx_p cy_p]); % 3 by 2
JdXdR = jacobian(X,[r11 r21 r31 r12 r22 r32 r13 r23 r33]); % 3 by 9
JdXdT = jacobian(X,[t1 t2 t3]); % 3 by 3
JdXdS = jacobian(X,[s1 s2 s3 r]); % 3 by 4

% camera projection

JdxdX = jacobian(x_c, [X1 X2 X3]); % 2 by 3
Jdxdfc = jacobian(x_c, [fx_c fy_c]); % 2 by 2
Jdxdcc = jacobian(x_c, [cx_c cy_c]); % 2 by 2
Jdxdkc = jacobian(x_c, [kc1 kc2 kc3 pc1 pc2]); % 2 by 5

% J_dxdp = [J_dxdfp J_dxdcp J_dxdR J_dxdT J_dxdS J_dxdfc J_dxdcc J_dxdkc];

%% Convert to m files
%  matlabFunction(JdxdX, 'File', 'JdxdX','Vars',[X1, X2, X3, fx_c, fy_c, cx_c, cy_c, kc1, kc2, kc3, pc1, pc2]);
%  matlabFunction(Jdxdfc, 'File', 'Jdxdfc','Vars',[X1, X2, X3, fx_c, fy_c, cx_c, cy_c, kc1, kc2, kc3, pc1, pc2]);
%  matlabFunction(Jdxdcc, 'File', 'Jdxdcc','Vars',[X1, X2, X3, fx_c, fy_c, cx_c, cy_c, kc1, kc2, kc3, pc1, pc2]);
%  matlabFunction(Jdxdkc, 'File', 'Jdxdkc','Vars',[X1, X2, X3, fx_c, fy_c, cx_c, cy_c, kc1, kc2, kc3, pc1, pc2]);
% 
% 
%  matlabFunction(JdXdfp, 'File', 'JdXdfp','Vars',[fx_p, fy_p, cx_p, cy_p, r11, r21, r31, r12, r22, r32, r13, r23, r33, t1, t2, t3, s1,s2,s3,r, x1_p,x2_p]);
%  matlabFunction(JdXdcp, 'File', 'JdXdcp','Vars',[fx_p, fy_p, cx_p, cy_p, r11, r21, r31, r12, r22, r32, r13, r23, r33, t1, t2, t3, s1,s2,s3,r, x1_p,x2_p]);
%  matlabFunction(JdXdR, 'File', 'JdXdR','Vars',[fx_p, fy_p, cx_p, cy_p, r11, r21, r31, r12, r22, r32, r13, r23, r33, t1, t2, t3, s1,s2,s3,r, x1_p,x2_p]);
%  matlabFunction(JdXdT, 'File', 'JdXdT','Vars',[fx_p, fy_p, cx_p, cy_p, r11, r21, r31, r12, r22, r32, r13, r23, r33, t1, t2, t3, s1,s2,s3,r, x1_p,x2_p]);
%  matlabFunction(JdXdS, 'File', 'JdXdS','Vars',[fx_p, fy_p, cx_p, cy_p, r11, r21, r31, r12, r22, r32, r13, r23, r33, t1, t2, t3, s1,s2,s3,r, x1_p,x2_p]);

%% Convert to c-code files

  ccode(JdxdX, 'File', 'JdxdX.c');
  ccode(Jdxdfc, 'File', 'Jdxdfc.c');
  ccode(Jdxdcc, 'File', 'Jdxdcc.c');
  ccode(Jdxdkc, 'File', 'Jdxdkc.c');
% 
% 
  ccode(JdXdfp, 'File', 'JdXdfp.c');
  ccode(JdXdcp, 'File', 'JdXdcp.c');
  ccode(JdXdR, 'File', 'JdXdR.c');
  ccode(JdXdT, 'File', 'JdXdT.c');
  ccode(JdXdS, 'File', 'JdXdS.c');
