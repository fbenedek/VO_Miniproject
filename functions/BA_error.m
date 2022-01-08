function error = BA_error(X_i, optimize_views_T_cw, optimize_X_indices, ...
    optimize_P_arrays, K)
% BA_ERROR calculates the bundle adjustment reprojection error
error = [];
for i = length(optimize_P_arrays)
    curr_X = X_i(:,optimize_X_indices{i});
    curr_error = reprojectionError(curr_X, optimize_P_arrays{i}, ...
        optimize_views_T_cw{i}, K);
    error = [error curr_error];
end
end

function e = reprojectionError(x,kpt,T_cw, K)
    T_cw = T_cw(1:3,:);
    X_i_C = T_cw*[x;ones(1,size(x,2))];
    projected_points = projectPoints(X_i_C, K);
    e = projected_points-kpt;
end 