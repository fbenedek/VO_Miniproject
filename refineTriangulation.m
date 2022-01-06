function X = refineTriangulation(P1,P2,X_0, T_cw1, T_cw2, K, params)
%REFINETRIANGULATION Summary of this function goes here
%   Detailed explanation goes here

error_function = @(x) reprojectionError(x,P1, P2,T_cw1, T_cw2, K);
options = optimoptions(@lsqnonlin,'Display','none','MaxIter', 20, 'Algorithm', 'levenberg-marquardt');
X = lsqnonlin(error_function, X_0, [], [], options);
end


function e = reprojectionError(x,kpt1, kpt2, T_cw1, T_cw2 , K)
    X_i_C1 = T_cw1*[x;ones(1,size(x,2))];
    X_i_C2 = T_cw2*[x;ones(1,size(x,2))];
    
    projected_points1 = projectPoints(X_i_C1, K);
    projected_points2 = projectPoints(X_i_C2, K);
    
    e1 = projected_points1-kpt1;
    e2 = projected_points2-kpt2;
    e = [e1;e2];
end 
