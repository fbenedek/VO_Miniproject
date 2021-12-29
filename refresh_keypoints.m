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
Cw_i = Rwc_i*[C_i; ones(1,M)]; %current keypoint vectors in world frame, 3*M

Fw_i = zeros(3,M); %first appearance keypoint vectors in world frame, 3*M
for i = 1:M
    R_Tau_i = [Tau_i(1:3,i), Tau_i(4:6,i), Tau_i(7:9,i)]; %first appearance rotation matrix, 3*3
    Fw_i(:,i) = R_Tau_i*[F_i(i); 1];
end

cos_angles = sum(Cw_i.*Fw_i)./(vecnorm(Cw_i).*vecnorm(Fw_i)); %cosine of angles using inner product, 1*M
bearing_angles = acos(cos_angles); %angles, 1*M

% select the indices which we will triangulate
triangulation_indices = bearing_angles > params.min_bearing_angle;
% triangulate points and get the new X_i, P_i values
X_new = triangulate_new_landmarks(C_i(:,triangulation_indices),...
    F_i(:,triangulation_indices), Tau_i(:,triangulation_indices), Twc_i, K);
X_i = [X_i, X_new];
P_i = [P_i, C_i(:,triangulation_indices)];
% discard the added points from C_i, F_i, Tau_i
C_i = C_i(:, ~triangulation_indices);
F_i = F_i(:, ~triangulation_indices);
Tau_i = Tau_i(:, ~triangulation_indices);

% set fields of the output S_i
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = C_i;
S_i.F_i = F_i;
S_i.Tau_i = Tau_i;
end