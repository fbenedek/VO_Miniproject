function S_i = get_new_canditates(image, S_i, params)
% GET_NEW_CANDIDATES returns a new state that contains a C_i to which new
% candidate points are added

% get fields from S_i to avoid visual clutter
C_i = S_i.C_i %...etc
% get Harris corners, parametrized by params
harris_corners = detectHarrisFeatures(image, 'MinQuality',params.harris_min_quality,...
    'FilterSize',params.harris_gaussian_dim);
% filter harris corners based on the distance from the current C_i and P_i,
% then adapt Mads' non-maximum surpression to arrive to the final list of
% proposals
proposed_points = filter_harris_corners(harris_corners, C_i, P_i, params);

% output the refreshed points
S_i.C_i = [S_i.C_i; proposed_points];

end