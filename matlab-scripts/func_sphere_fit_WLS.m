function [ Scenter, R_est, residue ] = func_sphere_fit_WLS( X_cam, W, isPlot  )
% Weighted linear least square fit for a 3D sphere using 3D points X_cam
% and weights W
% Input:
% - X_cam: 3D points on the sphere (3 x N)
% - W: weights in a diag matrix (N x N)
% Output:
% - Scenter: sphere center position (3 x 1)
% - R_est: sphere radius

%% WLS %%
b = -X_cam(1,:).^2-X_cam(2,:).^2-X_cam(3,:).^2;

beta = lscov([X_cam(1,:)' X_cam(2,:)' X_cam(3,:)' ones(length(X_cam),1)],b',W);

a_est = -beta(1)/2;
b_est = -beta(2)/2;
c_est = -beta(3)/2;
R_est = sqrt(beta(1)^2 + beta(2)^2 + beta(3)^2 -4*beta(4))/2;
Scenter = [a_est,b_est,c_est]';

%% plot fitting result %%
if isPlot
    figure,plot3(X_cam(1,:),X_cam(2,:),X_cam(3,:),'.b')
    hold on
    [x,y,z] = sphere(20);
    x1 = x*R_est  + a_est;
    y1 = y*R_est  + b_est;
    z1 = z*R_est  + c_est;
    mesh(x1,y1,z1,'Marker','.')
end
    vec = bsxfun(@minus, X_cam, Scenter);
    dist = bsxfun(@minus, sqrt(vec(1,:).^2 + vec(2,:).^2 +vec(3,:).^2), R_est);
    residue = sqrt(mean(dist.^2));

end

