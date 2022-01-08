function S = init_BA(S, P_0)
    S.BA_Locations = [0 0 0];
    S.BA_Orientations = reshape(eye(3),1,3,3);
    S.BA_Point_Tracks_i = {};
    S.BA_View_Ids = [1];
    S.BA_Candidate_Views_i = repmat([1],1,size(S.C_i,2));
    for i = 1:length(P_0)
        S.BA_Point_Tracks_i{end + 1} = [1];
    end
    S.BA_Point_Coordinates_i = {};
    for i = 1:length(P_0)
        S.BA_Point_Coordinates_i{end + 1} = P_0(:,i)';
    end
end