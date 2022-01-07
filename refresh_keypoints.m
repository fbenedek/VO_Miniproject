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

if params.use_simple_triangulation_criteria
    C_diff= C_i-F_i;
    C_dist = sqrt(sum((C_diff).^2,1));
    triangulation_indices = C_dist > 5 & C_dist < 300;
else
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
end

% triangulate points and get the new X_i, P_i values
[X_new, P_new] = triangulate_new_landmarks(C_i(:,triangulation_indices),...
    F_i(:,triangulation_indices), Tau_i(:,triangulation_indices), Twc_i, K, params);
X_i = [X_i, X_new];
P_i = [P_i, P_new];
% discard the added points from C_i, F_i, Tau_i
% Calculate the backprojection error for debug purposes

R_cw = Twc_i(1:3,1:3)';
t_cw = -R_cw*Twc_i(1:3,4);
T_cw = [R_cw, t_cw];

projection_matrix = K * T_cw;
avg_repro_err = avgReprojectionErr(projection_matrix,P_i,X_i);
fprintf("Average reprojection error is %f\n", avg_repro_err)
%reprojected_points - get them from cam matrix and current position!
% calculate previous and newly added repro error.
C_i = C_i(:, ~triangulation_indices);
F_i = F_i(:, ~triangulation_indices);
Tau_i = Tau_i(:, ~triangulation_indices);

if params.reproject_landmarks
    points_in_cur_frame = T_cw*[X_i;ones(1,size(X_i,2))];
    P_i = projectPoints(points_in_cur_frame, K);
end

fprintf("Added %i new keypoints!\n", size(X_new,2));
% set fields of the output S_i
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = C_i;
S_i.F_i = F_i;
S_i.Tau_i = Tau_i;
end