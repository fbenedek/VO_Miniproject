function [S, T_WC] = init_state(P_0, X_0, T_WC, img, params)
%INIT_STATE initialises the state struct. 
%   
% input:
%   P_0: initial keypoints found from bootstrap
%   X_0: initial landmarks found from bootstrap
%   T_WC: Coordinate transform from world to latest frame (boostrap frame)
%   (maybe for clarity this should be changed to T_CW and conversion to T_WC
%   can be performed locally in this function)
%   img: the latest image from boostrap, i.e. the one corresponding to the
%   keypoints in P_0
%
% output:
%   S: the state struct containing the markovian state
%       S.P=P_0 (2xM) initial keypoints
%       S.X=X_0 (3xM) initial landmarks
%       S.C (2xM) current keypoint candidates from img, these are different 
%       from those in P_0
%       S.F (2xM) first occurence of keypoint candidates (for init this is 
%       equal to S.C)
%       S.Tau (12xM) World to camera frame transformation (vectorized) for 
%       first occurence of candidate keypoints (for init these are all
%       T_WC)


hold off
c = double(getCornersSpread(img, [], params.num_initial_candidates, params).Location');
% figure
% imshow(img)
% hold on
% plot(c2)
% plot(cornerPoints(c'))
% hold off

S.P_i = P_0;
S.X_i  = X_0(1:3,:);
S.C_i  = c;
S.F_i  = c;
T_WC_vec = T_WC(:);
S.Tau_i = repmat(T_WC_vec, 1, size(c,2));
S.backup_frames = {};
S.backup_origins = {};
% init trackers
S.KLT_Point_Tracker = vision.PointTracker('NumPyramidLevels', ...
    params.point_tracker_levels, 'MaxBidirectionalError',params.point_tracker_max_error,...
    'BlockSize', [params.point_tracker_block_size,params.point_tracker_block_size]);
S.KLT_Candidate_Tracker = vision.PointTracker('NumPyramidLevels',...
    params.candidate_tracker_levels, 'MaxBidirectionalError',params.candidate_tracker_max_error,...
    'BlockSize', [params.candidate_tracker_block_size,params.candidate_tracker_block_size]);
initialize(S.KLT_Point_Tracker, S.P_i', img);
initialize(S.KLT_Candidate_Tracker, S.C_i', img);
% init bundle adjustment variables
S = init_BA(S, P_0);
end

