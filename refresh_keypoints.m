function S_i = refresh_keypoints(S_i, Twc_i, K, params)
% REFRESH_KEYPOINTS adds keypoints to P_i, X_i from C_i if their calculated
% bearing angle exceeds params.min_bearing_angle.

% get field of S_i to avoid unnecessary clutter
P_i = S_i.P_i;
X_i = S_i.X_i;
C_i = S_i.C_i;
F_i = S_i.F_i;
Tau_i = S_i.Tau_i;

M = size(C_i,2);
% calculate bearing_angles, an 1*M matrix containing bearing angles between
% each F_i,Tau_i (first view) pair and the current C_i, Twc_i
Rwc_i = Twc_i(1:3,1:3); %current rotation matrix, 3*3

Cw_i = Rwc_i*(K\[C_i; ones(1,M)]); %current keypoint vectors in world frame, 3*M

Fw_i = zeros(3,M); %first appearance keypoint vectors in world frame, 3*M
F_i_normalized = K\[F_i; ones(1,M)];
for i = 1:M
    R_Tau_i = [Tau_i(1:3,i), Tau_i(4:6,i), Tau_i(7:9,i)]; %first appearance rotation matrix, 3*3
    Fw_i(:,i) = R_Tau_i*F_i_normalized(:,i);
end

cos_angles = sum(Cw_i.*Fw_i)./(vecnorm(Cw_i).*vecnorm(Fw_i)); %cosine of angles using inner product, 1*M
bearing_angles = acos(cos_angles); %angles, 1*M

% select the indices which we will triangulate
triangulation_indices = bearing_angles > params.min_bearing_angle;
% triangulate points and get the new X_i, P_i values
[X_new, behind_cam_indices] = triangulate_new_landmarks(C_i(:,triangulation_indices),...
    F_i(:,triangulation_indices), Tau_i(:,triangulation_indices), Twc_i, K);
points_to_add = C_i(:,triangulation_indices);
% discard the added points from C_i, F_i, Tau_i
% Calculate the backprojection error for debug purposes
[avg_reprojection_error, errors] = get_reprojection_error(points_to_add, X_new, Twc_i, K);
% Exclude the new keypts that have a higher reprojection err than a
% threshold
faulty_keypts = vecnorm(errors,1) > 10;
indices_to_keep = ~behind_cam_indices & ~faulty_keypts;
X_i = [X_i, X_new(:,indices_to_keep)];
P_i = [P_i, points_to_add(:,indices_to_keep)];
fprintf("There are %i keypoints that have a higher error than 10. Removed them.\n", sum(faulty_keypts))
[avg_reprojection_error, errors] = get_reprojection_error(points_to_add(:,indices_to_keep), X_new(:,indices_to_keep), Twc_i, K);
fprintf("Average reprojection error of new points is %f\n", avg_reprojection_error)
%reprojected_points - get them from cam matrix and current position!
% calculate previous and newly added repro error.
C_i = C_i(:, ~triangulation_indices);
F_i = F_i(:, ~triangulation_indices);
Tau_i = Tau_i(:, ~triangulation_indices);

fprintf("Added %i new keypoints!\n", sum(indices_to_keep));
display(errors)
% set fields of the output S_i
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = C_i;
S_i.F_i = F_i;
S_i.Tau_i = Tau_i;
end