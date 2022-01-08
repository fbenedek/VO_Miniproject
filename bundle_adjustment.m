function S_i = bundle_adjustment(S_i, K, params)
% BUNDLE_ADJUSTMENT refines the trajectory and the triangulated points S_i
% with nonlinear optimization

% first assemble the arrays that we will optimize over
X_i = S_i.X_i;
BA_View_Ids = S_i.BA_View_Ids;
BA_Locations = S_i.BA_Locations;
BA_Orientations = S_i.BA_Orientations;
BA_Point_Tracks_i = S_i.BA_Point_Tracks_i;
BA_Point_Coordinates_i = S_i.BA_Point_Coordinates_i;
BA_max_length = params.ba_max_length;
% select the views to optimize
if length(BA_View_Ids) < BA_max_length
    optimized_views = BA_View_Ids;
    optimized_locations = BA_Locations;
    optimized_orientations = BA_Orientations;
else
    optimized_views = BA_View_Ids(end-BA_max_length + 1:end);
    optimized_locations = BA_Locations(end-BA_max_length + 1:end,:);
    optimized_orientations = BA_Orientations(end-BA_max_length + 1:end,:,:);
end
% index the needed keypoints
optimize_P_arrays = {};
optimize_X_indices = {};
optimize_views_T_cw = {};
for i = 1 : length(optimized_views)
    current_view_idx = optimized_views(i);
    current_P_array = [];
    current_X_indices = [];  
    current_view_T_cw = [reshape(optimized_orientations(i,:,:),3,3) ...
        optimized_locations(i,:)'; 0 0 0 1];
    optimize_views_T_cw{i} = current_view_T_cw;
    for j = 1 : length(X_i)
        % find if current point is projected
        current_point_track = BA_Point_Tracks_i{j};
        current_point_index = find(current_point_track == current_view_idx);
        % if point is projected, add it to the P_array and the X_indices
        if ~isempty(current_point_index)
            current_point_coords = BA_Point_Coordinates_i{j};
            current_P_array = [current_P_array current_point_coords(current_point_index,:)'];
            current_X_indices = [current_X_indices j];
        end
    end
    optimize_P_arrays{i} = current_P_array;
    optimize_X_indices{i} = current_X_indices;
end

error_terms = @(X_i) BA_error(X_i, optimize_views_T_cw, optimize_X_indices, ...
    optimize_P_arrays, K);
options = optimoptions(@lsqnonlin, 'Display','none','MaxIter', 20,...
    'Algorithm', 'levenberg-marquardt');
X_i = lsqnonlin(error_terms, X_i, [], [], options);
S_i.X_i = X_i;
end