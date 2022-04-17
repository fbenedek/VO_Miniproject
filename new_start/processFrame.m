function  [S_i, Twc_i] = process_frame(image, S_i, K, image_idx, params)
%PROCESS_FRAME Summary of this function goes here
%   Detailed explanation goes here

if params.use_histeq
    image = histeq(image);
end

P_i = S_i.P_i;
X_i = S_i.X_i;
C_i = S_i.C_i;
F_i = S_i.F_i;
Tau_i = S_i.Tau_i;
Location = S_i.Locations;
Orientation = S_i.Orientations;
ViewId = S_i.View_Ids
Prev_Pose_CW = S_i.Prev_Pose_CW ;
Candidate_Views_i = S_i.Candidate_Views_i;
Point_Tracks_i = S_i.Point_Tracks_i;
Point_Coordinates_i = S_i.Point_Coordinates_i;

track_score_thresh = 0.99;

% tracking triangulated points
setPoints(S_i.KLT_Point_Tracker, S_i.P_i'); % sets points in the last stored image
% store current pts for triangulation refinement
prev_P_i = P_i;
[P_i, point_validity, point_scores] = S_i.KLT_Point_Tracker(image);
tracking_mask = point_validity & (point_scores > track_score_thresh);
P_i = P_i(tracking_mask,:)';
% filter the stored pts as well
prev_P_i = prev_P_i(:,tracking_mask);
% filter bundle adjustment arrays too
Point_Tracks_i = Point_Tracks_i(tracking_mask);
Point_Coordinates_i = Point_Coordinates_i(tracking_mask);

X_i = X_i(:, tracking_mask);
fprintf('\n Tracking mask sum %d', sum(tracking_mask));

cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)],params.image_size);
[R_wc,t_wc,inlierIdx] = estimateWorldCameraPose(P_i', X_i',cameraParams, 'MaxReprojectionError', 2);
fprintf('\n inlierIdx sum %d', sum(inlierIdx));
R_CW_cur = R_wc';
t_CW_cur = -R_CW_cur*t_wc';
T_cw = [R_CW_cur, t_CW_cur];
%T_cw = refinePose(P_i(:,inlierIdx),X_i(:, inlierIdx), T_cw, K, params);

% Add the current camera pose and current inliers to the bundle arrays
Point_Tracks_i = Point_Tracks_i(inlierIdx);
Point_Coordinates_i = Point_Coordinates_i(inlierIdx);
for j = 1:length(Point_Tracks_i)
    Point_Tracks_i{j} = [Point_Tracks_i{j} image_idx];
end
for j = 1:length(Point_Coordinates_i)
    Point_Coordinates_i{j} = [Point_Coordinates_i{j}; P_i(:,j)'];
end

% tracking candidate points
setPoints(S_i.KLT_Candidate_Tracker, S_i.C_i'); % sets points in the last stored image
[C_i, point_validity, point_scores] = S_i.KLT_Candidate_Tracker(image);
tracking_mask = point_validity & (point_scores > track_score_thresh);
C_i = C_i(tracking_mask,:)';
F_i = F_i(:, tracking_mask);

% refine the triangulation based on the current and the previous frame

%S_i.X_i = refineTriangulationTwoView(prev_P_i(:, inlierIdx), P_i(:, inlierIdx), X_i(:, inlierIdx), S_i.Prev_Pose_CW, T_cw, K, params);

%% TRIANGULATION OF NEW POINTS
% points taken out for triangulatrion.
C_dist = sqrt(sum((C_i-F_i).^2,1));
C_dist_mask = C_dist > 10 & C_dist < 128;

Tau_triang = Tau_i(:, C_dist_mask);
C_triang = C_i(:,C_dist_mask);
F_triang = F_i(:, C_dist_mask);
Views_To_Add = Candidate_Views_i(C_dist_mask);

C_i = C_i(:, ~C_dist_mask);
F_i = F_i(:, ~C_dist_mask);
Tau_i = Tau_i(:, ~C_dist_mask);
Candidate_Views_i = Candidate_Views_i(:, ~C_dist_mask);


M2 = K * T_cw;

X_new = zeros(4,size(Tau_triang,2));
repro_err = zeros(1,size(Tau_triang,2));
for i = 1:size(Tau_triang,2)
    T_WC_tau = reshape(Tau_triang(:,i),3,4);
    R_CW_tau = T_WC_tau(1:3,1:3)';
    t_CW_tau = -R_CW_tau*T_WC_tau(:,end);
    M1 = K*[R_CW_tau, t_CW_tau];
    
    X_new(:,i) = linearTriangulation([F_triang(:,i);1],[C_triang(:,i);1],M1,M2);
    
    projection_matrix = K * T_cw;
    repro_err(i) = avgReprojectionErr(projection_matrix, C_triang(:,i), X_new(:,i));
end

X_new_in_cur_frame = [R_CW_cur, t_CW_cur]*X_new;
repro_err_mask = repro_err < 10 & X_new_in_cur_frame(3,:) > 0 & X_new_in_cur_frame(3,:) < 150;
X_new = X_new(1:3, repro_err_mask);
P_new = C_triang(:,repro_err_mask);
Views_To_Add = Views_To_Add(repro_err_mask);
Coords_To_Add = F_triang(:,repro_err_mask);
% Add new points to bundle adjusted structures
for j = 1 : length(Views_To_Add)
    Point_Tracks_i{end+1} = [Views_To_Add(j) image_idx];
    Point_Coordinates_i{end+1} = [Coords_To_Add(:,j)'; P_new(:,j)'];
end
% if length(X_new)>0
%     X_new = refineTriangulation(P_new,X_new,[R_CW_cur, t_CW_cur], K, params);
% end
%% UPDATE P_i, X_i
S_i.P_i = [P_i(:,inlierIdx), P_new];
S_i.X_i = [S_i.X_i(:,inlierIdx), X_new];

% %% BUNDLE ADJUSTMENT of poses and X_i
% add the current frame info
% intrinsics
intrinsics = cameraIntrinsics([K(1,1) K(2,2)],[K(1,3) K(2,3)], params.image_size);
% current points
xyzPoints = S_i.X_i;
% add current points to the pointTrack

% add the current pose to the poses
Location = [Location; t_wc];
Orientation = [Orientation; reshape(R_wc,1,3,3)];
ViewId = [ViewId; image_idx]; 
cameraPoses = table(ViewId, Orientation, Location);
% assemble point tracks
pointTracks = [];
for i = 1:length(Point_Tracks_i)
    pointTracks = [pointTracks pointTrack(Point_Tracks_i{i},Point_Coordinates_i{i})];
end
% bundle adjust
[xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints',pointTracks,cameraPoses,intrinsics);
% if more than ba_length frame, remove the oldest frame info

projection_matrix = K * T_cw;
%S_i.X_i = refineTriangulation(S_i.P_i,S_i.X_i,T_cw, K, params);
%repro_err = avgReprojectionErr(projection_matrix, S_i.P_i, S_i.X_i);
%fprintf("\n Average reprojection error: %i", repro_err)

%% FIND NEW KEYPOINT CANDIDATES
C_new = getCandidateCorners(image,S_i.P_i, params);
% flip to match convention
C_new = flipud(C_new);

%% UPDATE STATE
Twc_i = [R_wc, t_wc'];
S_i.C_i = [C_i, C_new];
S_i.F_i = [F_i, C_new];
S_i.Locations = Location;
S_i.Orientations = Orientation;
S_i.View_Ids = ViewId;
T_WC_vec = Twc_i(:);
S_i.Tau_i = [Tau_i, repmat(T_WC_vec, 1, size(C_new,2));];
S_i.Candidate_Views_i = [Candidate_Views_i, repmat([image_idx], 1, size(C_new,2))];
S_i.Prev_Pose_CW = T_cw;
S_i.Point_Tracks_i = Point_Tracks_i;
S_i.Point_Coordinates_i = Point_Coordinates_i;
end

