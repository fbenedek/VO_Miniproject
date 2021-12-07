function S_i, Twc_i = process_frame(image, prev_image, S_i)
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
% transformations from the first observation of each keypoint candidate to
% the current frame.
% Twc_i is 4*4 matrix containing the current pose, now the pose of the
% (can also be vectorized)
% Returns:
% Twc_i, a 4*4 matrix containing the current pose
% (can also be vectorized)
% S_i, the current state
end