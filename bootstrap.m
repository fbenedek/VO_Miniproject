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

% Detect Harris corners
points0 = detectHarrisFeatures(img0, 'MinQuality', min_harris_quality);
points1 = detectHarrisFeatures(img1, 'MinQuality', min_harris_quality);

% Extract features around Harris corners
[features0,valid_points0] = extractFeatures(img0,points0);
[features1,valid_points1] = extractFeatures(img1,points1);

% Match features
indexPairs = matchFeatures(features0,features1,'Unique', 1,'MatchThreshold',match_threshold);
matchedPoints0 = valid_points0(indexPairs(:,1),:);
matchedPoints1 = valid_points1(indexPairs(:,2),:);

% Estimate F
[F, inlier_idx] = estimateFundamentalMatrix(matchedPoints0,...
                    matchedPoints1,'Method','RANSAC','NumTrials', ransac_trials);

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
end
