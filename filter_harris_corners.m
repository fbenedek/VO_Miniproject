function proposed_points = filter_harris_corners(harris_corners, C_i, P_i, params)
% FILTER_HARRIS_CORNERS takes the harris corners that are at least
% params.min_new_point_distance (by norm params.proposal_test_norm) from
% any point from C_i and P_i, and then does nonMaxSurpression on them

% return a 2*K matrix of points to add to C_i
proposed_points = [];
end