function [X_new, behind_cam_indices] = triangulate_new_landmarks(C_i, F_i, Tau_i, Twc_i, K)
%TRIANGULATE NEW LANDMARKS triangulates new landmarks using current observations
%C_i and first observations F_i if the bearing angles exceed a given
%threshold
% C_i: current observations of candidates that will become keypoints (2*M)
% Twc_i: the current world frame (4*4)
% F_i: first observations of candidates that will become keypoints (2*M)
% Tau_i: the world frame of their first appearances (12*M)
% K: intrinsic camera matrix (3*3)

m = size(C_i,2);

p1 = [C_i; ones(1,m)]; %homogeneous img coordinates (current)

R_WC = Twc_i(1:3,1:3);
t_WC = Twc_i(1:3,4);
R_CW = R_WC';
t_CW = -R_CW*t_WC;
Rt_CW = [R_CW,t_CW]; %current [R|t] matrix, 3*4
M1 = K*Rt_CW; %current projection matrix, 3*4

p2 = [F_i; ones(1,m)]; %homogeneous img coordinates (first view)

X_new_homog = zeros(4,m); %result of linearTriangulation
diffs_from_matlab_triang = [0; 0; 0];
for i = 1:m
    R_WC_from_Tau = [Tau_i(1:3,i), Tau_i(4:6,i), Tau_i(7:9,i)]; %3*3
    t_WC_from_Tau = Tau_i(10:12,i);
    R_CW_from_Tau = R_WC_from_Tau';
    t_CW_from_Tau = -R_CW_from_Tau*t_WC_from_Tau;
    Rt_CW_from_Tau = [R_CW_from_Tau, t_CW_from_Tau];  %first view [R|t] matrix, 3*4
    M2 = K*Rt_CW_from_Tau; %first view projection matrix, 3*4
    X_new_homog(:,i) = [triangulate(p1(1:2,i)',p2(1:2,i)',M1',M2')'; 1];
        % currently implements triangulation 1-by-1 which is not ideal, but
        % M2 changes
end
avg_diff = mean(vecnorm(diffs_from_matlab_triang,1));
fprintf("The average absolute difference from matlab's triangulation is %f\n", avg_diff)
% TODO remove points that are triangulated behind the camera.
points_in_cam_coordinates = Rt_CW * X_new_homog;
behind_cam_indices = points_in_cam_coordinates(3,:) < 0;
fprintf("There are %i points behind the camera, removed them.\n", sum(behind_cam_indices))
X_new = X_new_homog;