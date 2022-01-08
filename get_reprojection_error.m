function [avg_reprojection_error, reprojection_errors] = get_reprojection_error(P_i, X_i, Twc_i, K)
% AVG_REPROJECTION_ERROR calculates the average reporojection error of
% S_i.X_i and S_i.P_i with camera matrix K from camera pose Twc_i

R_cw = Twc_i(1:3,1:3)';
t_cw = -R_cw*Twc_i(1:3,4);
T_cw = [R_cw, t_cw];
projection_matrix = K * T_cw;
projected_points = projection_matrix * X_i;
projected_points = projected_points./projected_points (3,:);
reprojection_errors = projected_points(1:2,:) - P_i;
avg_reprojection_error = double(mean(vecnorm(reprojection_errors,1)));