function [Rt_WC, P_i, X_i, BA_Point_Tracks_i, BA_Point_Coordinates_i, ...
    BA_Locations, BA_Orientations, BA_View_Ids] = ...
    get_pose(P_i, X_i, K, BA_Point_Tracks_i, BA_Point_Coordinates_i, ...
    image_index, BA_Locations, BA_Orientations, BA_View_Ids, params)
% GET_POSE returns the estimated camera pose as a 4x4 homogenous matrix
% from a p3p ransac algo. Set params.adaptive_ransac to 1 for adaptive and 
% 0 for regular ransac. The algorithm also chooses the right solution which
% has the points in from of the camera.
% needs the camera intrinsic matrix as params.K
% utilize the given p3p function with modifications

% Get pose
cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)],params.image_size);
[R_wc,t_wc,inlierIdx] = estimateWorldCameraPose(P_i', X_i',cameraParams, 'MaxReprojectionError', params.ransac_max_error);
fprintf('\n inlier ratio estimateWorldCameraPose %4.2f %%', 100*sum(inlierIdx)/max(1,size(inlierIdx,1)));

[rotationMatrix,translationVector] = cameraPoseToExtrinsics(R_wc,t_wc);
Rt_CW_matlab_conv = [rotationMatrix;translationVector];
Rt_CW = Rt_CW_matlab_conv';
Rt_CW = refinePose(P_i,X_i, Rt_CW, K, params);
Rt_WC = [Rt_CW(1:3,1:3)',-Rt_CW(1:3,1:3)'*Rt_CW(1:3,4)];

% Delete the indices of P_i and X_i that are outliers
P_i = P_i(:,inlierIdx);
X_i = X_i(:,inlierIdx);
% Do the same to the BA variables
BA_Point_Tracks_i = BA_Point_Tracks_i(inlierIdx);
BA_Point_Coordinates_i = BA_Point_Coordinates_i(inlierIdx);

% Then add current camera pose and feature coordinates to the BA vars
% Add pose and index
BA_Locations = [BA_Locations; Rt_CW(1:3,4)'];
BA_Orientations = [BA_Orientations; reshape(Rt_CW(1:3,1:3),1,3,3)];
% Add view index to track
BA_View_Ids = [BA_View_Ids; image_index];
% Add coordinates and camera index to points
for j = 1:length(BA_Point_Tracks_i)
    BA_Point_Tracks_i{j} = [BA_Point_Tracks_i{j} image_index];
end
for j = 1:length(BA_Point_Coordinates_i)
    BA_Point_Coordinates_i{j} = [BA_Point_Coordinates_i{j}; P_i(:,j)'];
end

end