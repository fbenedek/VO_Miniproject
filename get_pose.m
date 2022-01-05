function [Tcw_i, P_i, X_i] = get_pose(P_i, X_i, K, params)
% GET_POSE returns the estimated camera pose as a 4x4 homogenous matrix
% from a p3p ransac algo. Set params.adaptive_ransac to 1 for adaptive and 
% 0 for regular ransac. The algorithm also chooses the right solution which
% has the points in from of the camera.
% needs the camera intrinsic matrix as params.K
% utilize the given p3p function with modifications

% Get pose
 cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)], params.image_size);
 [worldOrientation,worldLocation, inlierIdx] = estimateWorldCameraPose(double(P_i)',...
     X_i',cameraParams, 'MaxNumTrials', 2500, "MaxReprojectionError",params.ransac_max_repro_err);
 R_C_W = worldOrientation';
 t_C_W = -R_C_W*worldLocation';
fprintf("Number of points dropped by pose ransac: %i", sum(~inlierIdx))
%[R_C_W, t_C_W, ~, ~, ~] = ransacLocalization(P_i, X_i, K, params);
% Convert to our fromat
Tcw_i = [R_C_W t_C_W; 0 0 0 1];
% Delete the indices of P_i and X_i that are outliers
P_i = P_i(:,inlierIdx);
X_i = X_i(:,inlierIdx);
end