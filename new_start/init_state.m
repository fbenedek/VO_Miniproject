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


% find keypoint candidates
params.use_shi_tomasi = 1;
if params.use_histeq
    img = histeq(img);
end
c = getCandidateCorners(img,P_0, params);
% flip to match convention
c = flipud(c);

S.P_i = P_0;
S.X_i  = X_0(1:3,:);
S.C_i  = c;
S.F_i  = c;
T_WC_vec = T_WC(:);
S.Tau_i = repmat(T_WC_vec, 1, size(c,2));
S.Candidate_Views_i = repmat([1],1,size(c,2));
% init trackers
S.KLT_Point_Tracker = vision.PointTracker();
S.KLT_Candidate_Tracker = vision.PointTracker();
initialize(S.KLT_Point_Tracker, S.P_i', img);
initialize(S.KLT_Candidate_Tracker, S.C_i', img);
S.KLT_Candidate_Tracker.BlockSize = [15,15];
S.KLT_Point_Tracker.BlockSize = [15,15];
S.KLT_Point_Tracker.MaxBidirectionalError = 1;
S.Locations = [0 0 0];
S.Orientations = reshape(eye(3),1,3,3);
S.Prev_Pose_CW = [eye(3), [0; 0; 0]];
S.Point_Tracks_i = {};
S.View_Ids = [1];
for i = 1:length(P_0)
    S.Point_Tracks_i{end + 1} = [1];
end
S.Point_Coordinates_i = {};
for i = 1:length(P_0)
    S.Point_Coordinates_i{end + 1} = P_0(:,i)';
end

