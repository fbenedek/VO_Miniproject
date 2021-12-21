function [R_C_W, t_C_W, best_inlier_mask, max_num_inliers_history, num_iteration_history] ...
    = ransacLocalization(matched_query_keypoints, corresponding_landmarks, params)
% query_keypoints should be 2x1000
% all_matches should be 1x1000 and correspond to the output from the
%   matchDescriptors() function from exercise 3.
% best_inlier_mask should be 1xnum_matched (!!!) and contain, only for the
%   matched keypoints (!!!), 0 if the match is an outlier, 1 otherwise.

% Read parameters
K = params.K;
num_iterations = params.ransac_num_iters; % number of iterations when not
% using adaptive ransac
adaptive = params.use_adaptive_ransac; % whether or not to use ransac adaptively
min_inlier_count = params.ransac_min_inlier_count;
pixel_tolerance = params.ransac_pixel_tolerance;
k = params.ransac_k; % number of pts needed for a model, 3 by default


if adaptive
    num_iterations = inf;
end

% Initialize RANSAC.
best_inlier_mask = zeros(1, size(matched_query_keypoints, 2));
% (row, col) to (u, v)
matched_query_keypoints = flipud(matched_query_keypoints);
max_num_inliers_history = [];
num_iteration_history = [];
max_num_inliers = 0;
% Replace the following with the path to your camera projection code:
% addpath('../../01_camera_projection/code');

% RANSAC
i = 1;

while num_iterations > i
    % Model from k samples (DLT or P3P)
    [landmark_sample, idx] = datasample(...
        corresponding_landmarks, k, 2, 'Replace', false);
    keypoint_sample = matched_query_keypoints(:, idx);
    
    
    % Backproject keypoints to unit bearing vectors.
    normalized_bearings = K\[keypoint_sample; ones(1, 3)];
    for ii = 1:3
        normalized_bearings(:, ii) = normalized_bearings(:, ii) / ...
            norm(normalized_bearings(:, ii), 2);
    end
    
    poses = p3p(landmark_sample, normalized_bearings);
    
    % Decode p3p output
    R_C_W_guess = zeros(3, 3, 4);
    t_C_W_guess = zeros(3, 1, 4);
    for ii = 0:3
        R_W_C_ii = real(poses(:, (2+ii*4):(4+ii*4)));
        t_W_C_ii = real(poses(:, (1+ii*4)));
        R_C_W_guess(:,:,ii+1) = R_W_C_ii';
        t_C_W_guess(:,:,ii+1) = -R_W_C_ii'*t_W_C_ii;
    end
    
    % Count inliers:
    projected_points = projectPoints(...
        (R_C_W_guess(:,:,1) * corresponding_landmarks) + ...
        repmat(t_C_W_guess(:,:,1), ...
        [1 size(corresponding_landmarks, 2)]), K);
    difference = matched_query_keypoints - projected_points;
    errors = sum(difference.^2, 1);
    is_inlier = errors < pixel_tolerance^2;
    
    % Consider inliers for the alternative solutions.
    
    for alt_idx=1:3
        projected_points = projectPoints(...
            (R_C_W_guess(:,:,1+alt_idx) * corresponding_landmarks) + ...
            repmat(t_C_W_guess(:,:,1+alt_idx), ...
            [1 size(corresponding_landmarks, 2)]), K);
        difference = matched_query_keypoints - projected_points;
        errors = sum(difference.^2, 1);
        alternative_is_inlier = errors < pixel_tolerance^2;
        if nnz(alternative_is_inlier) > nnz(is_inlier)
            is_inlier = alternative_is_inlier;
        end
    end
    
    
    if nnz(is_inlier) > max_num_inliers && ...
            nnz(is_inlier) >= min_inlier_count
        max_num_inliers = nnz(is_inlier);        
        best_inlier_mask = is_inlier;
    end
    
    if adaptive
        % estimate of the outlier ratio
        outlier_ratio = 1 - max_num_inliers / numel(is_inlier);
        % formula to compute number of iterations from estimated outlier
        % ratio
        confidence = params.ransac_confidence;
        upper_bound_on_outlier_ratio = params.ransac_UB_outlier;
        outlier_ratio = min(upper_bound_on_outlier_ratio, outlier_ratio);
        num_iterations = log(1-confidence)/log(1-(1-outlier_ratio)^k);
        % cap the number of iterations
        num_iterations = min(params.ransac_adaptive_cap, num_iterations);
    end
    
    num_iteration_history(i) = num_iterations;
    max_num_inliers_history(i) = max_num_inliers;
    
    i = i+1;
end

if max_num_inliers == 0
    R_C_W = [];
    t_C_W = [];
else
    M_C_W = estimatePoseDLT(...
        matched_query_keypoints(:, best_inlier_mask>0)', ...
        corresponding_landmarks(:, best_inlier_mask>0)', K);
    R_C_W = M_C_W(:, 1:3);
    t_C_W = M_C_W(:, end);
end

if adaptive
    disp(strcat("    Adaptive RANSAC: Needed ", num2str(i-1), " iteration to converge."));
    disp(strcat("    Adaptive RANSAC: Estimated Ouliers: ", num2str(int32(100*outlier_ratio)), "%"));
end

