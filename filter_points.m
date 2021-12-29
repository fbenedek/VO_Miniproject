function [P_i, P_prev, X_i] = filter_points(P_i, P_prev, X_i, point_scores, point_validity, params)
% FILTER_POINTS checks if the tracked points are still in the image and are
% tracked with a confidence that is greater than params.point_score_tresh
% params.image_size should be set to size(I) at the beginning of the
% script!
% The points that we fail to track are discarded from P_i, P_prev and X_i
% Test if points are in image
points_in_image = all([P_i > 0; P_i < [params.image_size(2); params.image_size(1)]],1);
point_valid_score = point_scores' > params.point_score_thresh;
point_validity = all([point_valid_score; points_in_image],1);
% apply indices
P_i = P_i(:,point_validity);
P_prev = P_prev(:,point_validity);
X_i = X_i(:, point_validity);
end