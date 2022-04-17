function T_cw = refinePose(P,X_0, T_cw_0, K, params)
%REFINEPOSES Summary of this function goes here
%   Detailed explanation goes here

twist_cw_0 = HomogMatrix2twist([T_cw_0;0,0,0,1]);

error_function = @(T) reprojectionError(X_0,P,T, K);
options = optimoptions(@lsqnonlin,'Display','none','MaxIter', 20, 'Algorithm', 'levenberg-marquardt');
twist_cw = lsqnonlin(error_function, twist_cw_0, [], [], options);
T_cw = twist2HomogMatrix(twist_cw);
T_cw = T_cw(1:3,:);
end


function e = reprojectionError(x,kpt,tw_cw, K)
    T_cw = twist2HomogMatrix(tw_cw);
    T_cw = T_cw(1:3,:);
    X_i_C = T_cw*[x;ones(1,size(x,2))];
    projected_points = projectPoints(X_i_C, K);
    e = projected_points-kpt;
end 

