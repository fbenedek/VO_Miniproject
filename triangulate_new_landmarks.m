function [X_new, P_new, repro_err_mask] = triangulate_new_landmarks(C_i, F_i, Tau_i, Rt_WC, K, params)
%TRIANGULATE NEW LANDMARKS triangulates new landmarks using current observations
%C_i and first observations F_i if the bearing angles exceed a given
%threshold
% C_i: current observations of candidates that will become keypoints (2*M)
% Twc_i: the current world frame (4*4)
% F_i: first observations of candidates that will become keypoints (2*M)
% Tau_i: the world frame of their first appearances (12*M)
% K: intrinsic camera matrix (3*3)

m = size(C_i,2);

R_WC = Rt_WC(1:3,1:3);
t_WC = Rt_WC(1:3,4);
R_CW = R_WC';
t_CW = -R_CW*t_WC;
Rt_CW = [R_CW,t_CW]; %current [R|t] matrix, 3*4
M2 = K * Rt_CW;

X_new = zeros(3,m); %result of linearTriangulation
repro_err = zeros(1,m);
for i = 1:m
    T_WC_tau = reshape(Tau_i(:,i),3,4);
    R_CW_tau = T_WC_tau(1:3,1:3)';
    t_CW_tau = -R_CW_tau*T_WC_tau(:,end);
    T_cw_tau = [R_CW_tau, t_CW_tau];
    M1 = K*T_cw_tau;
    
    X_new(1:3,i) = triangulate(F_i(:,i)',C_i(:,i)',M1',M2');
    
    X_new(1:3,i) = refineTriangulation(F_i(:,i),C_i(:,i),...
                        X_new(1:3,i),T_cw_tau, Rt_CW, K, params);
    repro_err(i) = avgReprojectionErr(M2, C_i(:,i), X_new(:,i));
end

X_new_in_cur_frame = Rt_CW*[X_new;ones(1,m)];
repro_err_mask = repro_err < params.max_rp_err_for_triangulation...
    & X_new_in_cur_frame(3,:) > 0 & X_new_in_cur_frame(3,:) < 500;
X_new = X_new(1:3, repro_err_mask);
P_new = C_i(:,repro_err_mask);