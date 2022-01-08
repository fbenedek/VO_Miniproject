function [Rt_WC, S_i] = track_and_get_pose(image, S_i, K, params)
% TRACK_AND_GET_POSE track the keypoints and candidates P_i, C_i with KLT
% trackers KLT_Point_Tracker and KLT_Candidate_Tracker
% gets the camera pose Twc_i with a p3p (adaptive) ransac

% get fields for less visual clutter
P_i = S_i.P_i;
X_i = S_i.X_i;
C_i = S_i.C_i;
F_i = S_i.F_i;
Tau_i = S_i.Tau_i;
BA_Locations = S_i.BA_Locations;
BA_Orientations = S_i.BA_Orientations;
BA_Point_Tracks_i = S_i.BA_Point_Tracks_i;
BA_View_Ids = S_i.BA_View_Ids;
BA_Point_Coordinates_i = S_i.BA_Point_Coordinates_i;
BA_Candidate_Views_i = S_i.BA_Candidate_Views_i;
image_index = S_i.image_idx;

KLT_Point_Tracker = S_i.KLT_Point_Tracker;
KLT_Candidate_Tracker = S_i.KLT_Candidate_Tracker;

% track the keypoints and candidates
% first the keypoints
setPoints(KLT_Point_Tracker, P_i');
% track the keypoints, storing their previous position
P_prev = P_i;
[P_i, point_validity, point_scores] = KLT_Point_Tracker(image);
% drop the keypoints that are: out of the image OR have a lower score than
% params.track_score_thresh
% Transpose P_i to keep conventions
P_i = P_i';
[P_i, P_prev, X_i, BA_Point_Tracks_i, BA_Point_Coordinates_i] = ...
    filter_points(P_i, P_prev, X_i, point_scores, point_validity, ...
    BA_Point_Tracks_i, BA_Point_Coordinates_i, params);

% figure for debugging
figure(5)
imshow(image)
hold on
y_from = P_prev(2,:);
x_from = P_prev(1,:);
y_to = P_i(2,:);
x_to = P_i(1,:);
figure(5)
plot([x_from; x_to],[y_from; y_to], 'g-', 'Linewidth', 2);

% do RANSAC p3p and get pose
% NOTE, get_pose seemingly outputs Tcw, not Twc.
[Rt_WC, P_i, X_i, BA_Point_Tracks_i, BA_Point_Coordinates_i, ...
    BA_Locations, BA_Orientations, BA_View_Ids] = ...
    get_pose(P_i, X_i(1:3,:), K, BA_Point_Tracks_i, ...
    BA_Point_Coordinates_i, image_index, BA_Locations, ...
    BA_Orientations, BA_View_Ids ,params);

% also for figure
plot(P_i(1,:),P_i(2,:), 'go', 'Linewidth', 2);
hold off

if C_i
    % move on to the candidates
    setPoints(KLT_Candidate_Tracker, C_i');
    [C_i, candidate_validity, candidate_scores] = KLT_Candidate_Tracker(image);
    % Transpose C_i to keep conventions
    C_i = C_i';
    % filter similarly to the keypoints
    [C_i, F_i, Tau_i, BA_Candidate_Views_i] = filter_candidates(C_i, F_i, Tau_i, candidate_scores,...
        candidate_validity, BA_Candidate_Views_i, params);
end

% set the output dict
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = C_i;
S_i.F_i = F_i;
S_i.Tau_i = Tau_i;
S_i.KLT_Point_Tracker = KLT_Point_Tracker;
S_i.KLT_Candidate_Tracker = KLT_Candidate_Tracker;
S_i.BA_Point_Tracks_i = BA_Point_Tracks_i;
S_i.BA_Point_Coordinates_i = BA_Point_Coordinates_i; 
S_i.BA_Locations = BA_Locations;
S_i.BA_Orientations = BA_Orientations;
S_i.BA_View_Ids = BA_View_Ids;
end