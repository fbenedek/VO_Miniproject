function S_i, Twc_i = init_state(P_0, X_0, img0, img1)
% INIT_STATE initializes the continous operation of the VO pipeline
% Arguments:
% P_0 is a 2*K matrix containing the 2D positions of the keypoints on img0
% X_0 is a 3*K matrix containing the 3D positions of the keypoints
% img0 and img1, the images used for bootstrapping
% Returns:
% Twc_i is 4*4 matrix containing the current pose, which is the pose of the
% frame of img1 (can also be vectorized)
% S_i, a struct, with the following fields:
% P_i is a 2*K matrix containing the 2D positions of the keypoints on img1
% X_i is a 3*K matrix containing the 3D positions of the keypoints
% C_i is a 2*M matrix containing keypoints candidates from img1, which are
% not similar to P_i but are Harris corners
% F_i is a 2*M matrix and stores the first observations of new keypoint
% candidates. Identical to C_i.
% Tau_i is a 12*M matrix containing the concatenated and vectorized
% transformations from the first observation of each keypoint candidate to
% the current frame. Contains M identity transformations only.
end