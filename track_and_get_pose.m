function [Twc_i, S_i] = track_and_get_pose(image, S_i, K, params)
% TRACK_AND_GET_POSE track the keypoints and candidates P_i, C_i with KLT
% trackers KLT_Point_Tracker and KLT_Candidate_Tracker
% gets the camera pose Twc_i with a p3p (adaptive) ransac

% get fields for less visual clutter
P_i = S_i.P_i;
X_i = S_i.X_i;
C_i = S_i.C_i;
F_i = S_i.F_i;
Tau_i = S_i.Tau_i;
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
[P_i, P_prev, X_i] = filter_points(P_i, P_prev, X_i, point_scores, point_validity, params);


% move on to the candidates
setPoints(KLT_Candidate_Tracker, C_i');
[C_i, ~, candidate_scores] = KLT_Candidate_Tracker(image);
% Transpose C_i to keep conventions
C_i = C_i';
% filter similarly to the keypoints
[C_i, F_i, Tau_i] = filter_candidates(C_i, F_i, Tau_i, candidate_scores, params);


% do RANSAC p3p and get pose
% NOTE, get_pose seemingly outputs Tcw, not Twc.
Tcw_i = get_pose(P_i, X_i(1:3,:), K, params);
Rcw = Tcw_i(1:3,1:3);
t_cw = Tcw_i(1:3,end);
Rwc = Rcw';
t_wc = -Rwc*t_cw;
Twc_i = [Rwc, t_wc; 0 0 0 1];

% set the output dict
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = C_i;
S_i.F_i = F_i;
S_i.Tau_i = Tau_i;
S_i.KLT_Point_Tracker = KLT_Point_Tracker;
S_i.KLT_Candidate_Tracker = KLT_Candidate_Tracker;


end