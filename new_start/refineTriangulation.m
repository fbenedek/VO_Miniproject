function X = refineTriangulation(P,X_0, T_cw, K, params)
%REFINETRIANGULATION Obtains a refined pointcloud from the past view
%   Detailed explanation goes here

error_function = @(x) sum(vecnorm(reprojectionError(x,P,T_cw, K),1));
options = optimoptions(@lsqnonlin,'Display','none','MaxIter', 20, 'Algorithm', 'levenberg-marquardt');
X = lsqnonlin(error_function, X_0, [], [], options);
end



function e = reprojectionError(x,kpt,T_cw, K)
    X_i_C = T_cw*[x;ones(1,size(x,2))];
    projected_points = projectPoints(X_i_C, K);
    e = projected_points-kpt;
end 
