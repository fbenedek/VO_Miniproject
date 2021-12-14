function S_i = refresh_keypoints(S_i, Twc_i, params)
% REFRESH_KEYPOINTS adds keypoints to P_i, X_i from C_i if their calculated
% bearing angle exceeds params.min_bearing_angle.

% get field of S_i to avoid unnecessary clutter
P_i = S_i.P_i; %..etc

% calculate bearing_angles, an 1*M matrix containing bearing angles between
% each F_i,Tau_i (first view) pair and the current C_i, Twc_i
bearing_angles = ... ;
% select the indices which we will triangulate
triangluation_indices = bearing_angles > params.min_bearing_angle;
% triangulate points and get the new X_i, P_i values
X_new = triangulate_points(C_i, F_i, Tau_i, Twc_i, triangluation_indices);
X_i = [X_i X_new];
P_i = [P_i, C_i(:,triangluation_indices)]
% discard the added points from C_i, F_i, Tau_i
C_i = C_i(:, ~triangluation_indices);
F_i = F_i(:, ~triangluation_indices);
Tau_i = Tau_i(:, ~triangluation_indices);

% set fields of the output S_i
S_i.P_i = P_i %...etc
end