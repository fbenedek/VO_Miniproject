function [P_0, X_0, T_WC] = bootstrap(img0, img1, K, params)
% BOOTSTRAP return keypoints and landmarks for initialisation as well as
% camera frame transformation computed in the process.
%
% input:
%   img0: first image in the sequence
%   img2: image later in sequence to be used for bootstrap.
%
%Returns
%   P_0, a 2*M matrix of image coordinates on img1
%   X_0, a 3*M matrix of 3D positions, the triangulated pointcloud (the
%   camera frame from img0 is considered as the world frame
%   T_WC, transformation from world frame (img0) to camera frame from img1

avg_repro_err = 10;

while avg_repro_err > params.bootstrap_repro_error

    kp0 = getCornersSpread(img0, [], params.points_num_target,params);
    shw_pt0 = double(kp0.Location');

    KLT_Point_Tracker = vision.PointTracker('NumPyramidLevels', params.point_tracker_levels, 'MaxBidirectionalError',5);
    initialize(KLT_Point_Tracker, shw_pt0', img0);
    
    [shw_pt1, point_validity, point_scores] = KLT_Point_Tracker(img1);
    shw_pt1 = shw_pt1';
    kp1 = shw_pt1(:,point_validity);
    kp0 = shw_pt0(:,point_validity);

%     imshow(img1)
%     hold on;
%     plot(shw_pt1(1, :), shw_pt1(2, :), 'rx', 'Linewidth', 2);
%     plot(shw_pt0(1, :), shw_pt0(2, :), 'bx', 'Linewidth', 2);
%     y_from = kp0(2, :);
%     x_from = kp0(1, :);
%     y_to = kp1(2,:);
%     x_to = kp1(1,:);
%     plot([x_from; x_to],[y_from; y_to], 'g-', 'Linewidth', 2);

    [F,inliersIndex] = estimateFundamentalMatrix(kp0',kp1', 'Method','RANSAC', 'DistanceThreshold',0.1);
%     plot(kp1(1, inliersIndex), kp1(2, inliersIndex), 'ro', 'Linewidth', 2);
%     hold off;

    cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)],params.image_size);
    kp0 = kp0(:, inliersIndex); kp1 = kp1(:, inliersIndex);

    [relativeOrientation,relativeLocation, validRatio] = relativeCameraPose(F,cameraParams,kp0',kp1');
    Rt_WC1_matlab_conv = [relativeOrientation;relativeLocation];
    Rt_WC1 = Rt_WC1_matlab_conv';

    [rotationMatrix,translationVector] = cameraPoseToExtrinsics(relativeOrientation,relativeLocation);
    Rt_C1W_matlab_conv = [rotationMatrix;translationVector];
    Rt_C1W = Rt_C1W_matlab_conv';

    M1 = K * eye(3,4);
    M2 = K * Rt_C1W;
    %X_0 = linearTriangulation([kp0;ones(1, size(kp0,2))],[kp1; ones(1, size(kp1,2))],M1,M2);
    X_0 = triangulate(kp0',kp1', M1', M2')';
    X_0 = refineTriangulation(kp0,kp1,...
                            X_0,eye(3,4), Rt_C1W, K, params);              
    X_0_mask = X_0(3,:) > 0 & X_0(3,:) < params.depth_thresh*median(X_0(3,:));
    X_0 = X_0(:, X_0_mask);
    P_0 = kp1(:, X_0_mask);

    T_WC = Rt_WC1;
    projection_matrix = K * Rt_C1W;
    projected_points = projection_matrix * [X_0; ones(1,size(X_0,2))];
    projected_points = projected_points./projected_points (3,:);
    reprojection_errors = projected_points(1:2,:) - P_0;
    avg_repro_err = mean(vecnorm(reprojection_errors,1));
    fprintf('\n bootstrap reprojection error %d', avg_repro_err);
end % end while

end
