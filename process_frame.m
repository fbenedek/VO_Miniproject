function [S_i, Twc_i] = process_frame(image, S_i, K, params)
% PROCESS_FRAME iterates the VO pipeline by processing a new frame.
% Arguments:
% Current and previous images (image, prev_image)
% S_i, the previous state, which is a struct with fields:
% P_i is a 2*K matrix containing the 2D positions of the keypoints on
% prev_img
% X_i is a 3*K matrix containing the 3D positions of the keypoints
% C_i is a 2*M matrix containing keypoints candidates from prev_img, which are
% not similar to P_i but are Harris corners
% F_i is a 2*M matrix and stores the first observations of new keypoint
% candidates.
% Tau_i is a 12*M matrix containing the concatenated and vectorized
% global coordinates of the first observation poses for each candidate
% KLT_Tracker, KLT_Candidate_Tracker: visionTracker objects that handle the
% tracking of keypoints and keypoint candidates separately
% -- end of fields --
% Twc_i is 4*4 matrix containing the current pose
% (can also be vectorized)
% params, a struct that contains the parameters
% Returns:
% Tcw_i, a 4*4 matrix containing the current pose
% (can also be vectorized)
% S_i, the current state

% 4.1 and 4.2: Track keypoints + candidates and get camera pose and filtered keypoints
% from p3p ransac
[Twc_i, S_i] = track_and_get_pose(image, S_i, K, params);

% 4.3 triangulate and add new points if possible
% triangulate the points that have a sufficient bearing angle and add them
% to the keypoints
S_i = refresh_keypoints(S_i, Twc_i, K, params);
% get new possible keypoints - the Harris corners that do not coincide with
% the current C_i or P_i
S_i = get_new_canditates(image, S_i, Twc_i, params);


end