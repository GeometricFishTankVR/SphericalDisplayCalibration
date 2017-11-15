function [ Rp1, Tp1 ] = func_compute_extrin( p1, p2, F, fc_cam,cc_cam,cam_dist,fc_proj,cc_proj,proj_dist )
% Compute extrinsic rotation and translation of a stereo pair with known
% intrinsic focal lengths, principle points and lens distortions
% return:
% - Extrinsic rotation Rp1(3 x 3)
% - Extrinsic translation Tp1 (3 x 1)

[U,~,V] = svd(F);
W = [0 -1 0;1 0 0;0 0 1];
R1 = U*W*V';
R2 = U*W'*V';
T = U*[0 0 1]';

R1 = R1 * sign(det(R1));
R2 = R2 * sign(det(R2));

om1 = rodrigues(R1);
om2 = rodrigues(R2);

T1 = T;
T2 = -T;

[Xc_1_cam,Xc_1_proj] = stereo_triangulation(p1,p2,om1,T1,fc_cam,cc_cam,cam_dist,0,fc_proj,cc_proj,proj_dist,0);
[Xc_2_cam,Xc_2_proj] = stereo_triangulation(p1,p2,om1,T2,fc_cam,cc_cam,cam_dist,0,fc_proj,cc_proj,proj_dist,0);
[Xc_3_cam,Xc_3_proj] = stereo_triangulation(p1,p2,om2,T1,fc_cam,cc_cam,cam_dist,0,fc_proj,cc_proj,proj_dist,0);
[Xc_4_cam,Xc_4_proj] = stereo_triangulation(p1,p2,om2,T2,fc_cam,cc_cam,cam_dist,0,fc_proj,cc_proj,proj_dist,0);

for i=1:4
    if eval(['mean(sign(Xc_' num2str(i) '_cam(3,:))) ==1 && mean(sign(Xc_' num2str(i) '_proj(3,:))) ==1']) 
        sol_num = i;        
    end
end

%eval(['C_X_cam1 = Xc_' num2str(sol_num) '_cam(:,1:length(p1));']);

switch sol_num
    case 1
        Rp1 = rodrigues(om1);
        Tp1 = T1;
    case 2
        Rp1 = rodrigues(om1);
        Tp1 = T2;
    case 3
        Rp1 = rodrigues(om2);
        Tp1 = T1;
    case 4
        Rp1 = rodrigues(om2);
        Tp1 = T2;       
end

end

