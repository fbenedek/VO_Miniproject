function [T_cw, Kpt] = refinePoseAndKpt(P,X_0, T_cw_0, K, params)
%REFINEPOSES Summary of this function goes here
%   Detailed explanation goes here
twist_cw_0 = HomogMatrix2twist([T_cw_0;0,0,0,1]);
kpt_unroll = P;
kpt_unroll = kpt_unroll(:);
P_T_0 = [twist_cw_0;kpt_unroll];

error_function = @(P_T) reprojectionError(X_0,P_T, K);
options = optimoptions(@lsqnonlin,'Display','none','MaxIter', 20, 'Algorithm', 'levenberg-marquardt');
P_T = lsqnonlin(error_function, P_T_0, [], [], options);

T_cw = twist2HomogMatrix(P_T(1:6));
T_cw = T_cw(1:3,:);
Kpt = reshape(P_T(7:end),2,[]);
end

function e = reprojectionError(x,p_t, K)
    tw_cw = p_t(1:6);
    T_cw = twist2HomogMatrix(tw_cw);
    T_cw = T_cw(1:3,:);
    kpt = reshape(p_t(7:end),2,[]);
    X_i_C = T_cw*[x;ones(1,size(x,2))];
    projected_points = projectPoints(X_i_C, K);
    e = projected_points-kpt;
end 
