function [S, T_WC] = init_state(P_0, X_0, T_WC, img)
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
c = getNewCandidateKeypoints(img,P_0);

S.P = P_0;
S.X = X_0;
S.C = c;
S.F = c;
T_WC_vec = T_WC(:);
S.Tau = repmat(T_WC_vec, 1, size(c,2));
end

