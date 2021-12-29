function S_i = get_new_canditates(image, S_i, Twc_i, params)
% GET_NEW_CANDIDATES returns a new state that contains a C_i to which new
% candidate points are added

% get fields from S_i to avoid visual clutter

P_i = S_i.P_i;
X_i = S_i.X_i;
C_i = S_i.C_i;
F_i = S_i.F_i;
Tau_i = S_i.Tau_i;

% get Harris corners, parametrized by params
harris_corners = detectHarrisFeatures(image, 'MinQuality',params.harris_min_quality,...
    'FilterSize',params.harris_gaussian_dim);
% filter harris corners based on the distance from the current C_i and P_i,
% then adapt Mads' non-maximum surpression to arrive to the final list of
% proposals
proposed_points = filter_harris_corners(harris_corners, C_i, P_i, params);

curr_Tau = [Twc_i(1:3,1); Twc_i(1:3,2); Twc_i(1:3,3); Twc_i(1:3,4)];
%1*12 vectorized current Twc_i

proposed_Tau = repmat(curr_Tau, 1, size(proposed_points,2));
%repmat of curr_Tau (as all new points have the same Tau)

% output the refreshed points
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = [C_i, proposed_points];
S_i.F_i = [F_i, proposed_points];
S_i.Tau_i = [Tau_i, proposed_Tau];

end