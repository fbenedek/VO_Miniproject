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

match_threshold = params.initial_match_threshold;
ransac_trials = params.initial_ransac_trials;
depth_threshold_multiplier = params.initial_depth_threshold_multiplier;
min_harris_quality = params.min_harris_feature_quality;

% img0 = histeq(img0);
% img1 = histeq(img1);

% Detect Harris corners
points0 = detectHarrisFeatures(img0, 'MinQuality', min_harris_quality, 'FilterSize',7);
points1 = detectHarrisFeatures(img1, 'MinQuality', min_harris_quality, 'FilterSize', 7);

% points0 = detectORBFeatures(img0);
% points1 = detectORBFeatures(img1);
% 
points0 = detectFASTFeatures(img0);
points1 = detectFASTFeatures(img1);
% 
% points0 = detectMinEigenFeatures(img0, 'MinQuality', min_harris_quality, 'FilterSize',7);
% points1 = detectMinEigenFeatures(img1, 'MinQuality', min_harris_quality, 'FilterSize',7);

% init_tracker = vision.PointTracker('MaxBidirectionalError',1);
% initialize(init_tracker,points0.Location,img0);
% [points1,validity] = init_tracker(img1);

% imshow(img1); hold on;
% plot(points1);
% 
% % Extract features around Harris corners
[features0,valid_points0] = extractFeatures(img0,points0);
[features1,valid_points1] = extractFeatures(img1,points1);
% 
% % Match features
indexPairs = matchFeatures(features0,features1,'Unique', 1,'MatchThreshold',match_threshold);
matchedPoints0 = valid_points0(indexPairs(:,1),:);
matchedPoints1 = valid_points1(indexPairs(:,2),:);

% imshow(img1); hold on;
% plot(matchedPoints1);
% 
% matchedPoints0 = points0.Location(validity,:);
% matchedPoints1 = points1(validity,:);


% Estimate F
[F, inlier_idx] = estimateFundamentalMatrix(matchedPoints0,...
                    matchedPoints1,'Method','RANSAC','NumTrials', ransac_trials);

cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)],params.image_size);
[relativeOrientation,relativeLocation] = relativeCameraPose(F,cameraParams,matchedPoints0(inlier_idx,:),matchedPoints1(inlier_idx,:));
          
% Get E from F
E = K'*F*K;

% Find the most likely transformation 
[Rots, u3] = decomposeEssentialMatrix(E);
kp0 = matchedPoints0.Location(inlier_idx,:); kp0 = [kp0, ones(size(kp0,1),1)];
kp1 = matchedPoints1.Location(inlier_idx,:); kp1 = [kp1, ones(size(kp1,1),1)];
[R_CW, T_CW] = disambiguateRelativePose(Rots, u3, kp0', kp1', K, K);

% Create M matrices and trinagulate points

M1 = K * eye(3,4);
M2 = K * [R_CW, T_CW];

X_0 = linearTriangulation(kp0', kp1', M1, M2);
depth_median = median(X_0(3,:));
depth_mask = (X_0(3,:) < depth_threshold_multiplier*depth_median) & (X_0(3,:) > 0); % probably not a great way to do this
X_0 = X_0(:, depth_mask);
P_0 = kp1(:,1:2)';
P_0 = P_0(:, depth_mask);
R_WC = R_CW';
t_WC = -R_WC*T_CW;
T_WC = [R_WC, t_WC];

projection_matrix = K * [R_CW, T_CW];
projected_points = projection_matrix * X_0;
projected_points = projected_points./projected_points (3,:);
reprojection_errors = projected_points(1:2,:) - P_0;
avg_repro_err = mean(vecnorm(reprojection_errors,1));

end
