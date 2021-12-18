function proposed_points = filter_harris_corners(harris_corners, C_i, P_i, params)
% FILTER_HARRIS_CORNERS takes the harris corners that are at least
% params.min_new_point_distance (by norm params.proposal_test_norm) from
% any point from C_i and P_i, and then does nonMaxSurpression on them
% Marci: do it in the opposite order (nonMax, then distance-based filtering
%
% return a 2*K matrix of points to add to C_i

max_points = nonMaxSupression(harris_corners, 10);

concat = [P_i, C_i];
%distances = pdist2(concat', max_points.Location, 'cityblock', 'Smallest', 1);
distances = pdist2(concat', max_points.Location, params.proposal_test_norm, 'Smallest', 1);
distinct_points_indices = distances > params.min_new_point_distance;

proposed_points = max_points.Location(distinct_points_indices, :)'; %2*n matrix of distinct points

end