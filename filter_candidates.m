function [C_i, F_i, Tau_i] = filter_candidates(C_i, F_i, Tau_i, candidate_scores, params)
% FILTER_CANDIDATES checks if candidate points are still in the image and
% if they have sufficiently low SSD distance from the previous frame. For
% the latter, the threshold params.candidate_score_thresh can be set.
% params.image_size should be set to size(I) at the beginning of the
% script!
% The points that we fail to track are discarded from C_i, F_i and Tau_i
% Test if points are in image
candidates_in_image = all([C_i > 0; C_i <  [params.image_size(2); params.image_size(1)]],1);
candidate_valid_score = candidate_scores' > params.candidate_score_thresh;
candidate_validity = all([candidate_valid_score; candidates_in_image],1);
% apply indices
C_i = C_i(:,candidate_validity);
F_i = F_i(:, candidate_validity);
Tau_i = Tau_i(:, candidate_validity);
end