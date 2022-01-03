function [Tcw_i] = get_pose(P_i, X_i, K, params)
% GET_POSE returns the estimated camera pose as a 4x4 homogenous matrix
% from a p3p ransac algo. Set params.adaptive_ransac to 1 for adaptive and 
% 0 for regular ransac. The algorithm also chooses the right solution which
% has the points in from of the camera.
% needs the camera intrinsic matrix as params.K
% utilize the given p3p function with modifications

% Get pose
 cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)],params.image_size);
 [worldOrientation,worldLocation] = estimateWorldCameraPose(double(P_i)',...
     X_i',cameraParams, 'MaxNumTrials', 2500);
 R_C_W = worldOrientation';
 t_C_W = -R_C_W*worldLocation';
%[R_C_W, t_C_W, ~, ~, ~] = ransacLocalization(P_i, X_i, K, params);
% Convert to our fromat
Tcw_i = [R_C_W t_C_W; 0 0 0 1];
end