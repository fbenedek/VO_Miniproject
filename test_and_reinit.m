function S_i = test_and_reinit(image, S_i, Rt_WC, K, params)
% TEST_AND_REINIT backs up the last params.re_init_frame_distance frames,
% and reinitializes the VO pipeline if the number of tracked points drop
% below params.re_init_min_kps. Set this minimum such that two inits do not
% happen closer than params.re_init_frame_distance frames from each other.

if length(S_i.BA_View_Ids) <= params.re_init_frame_distance
    S_i.backup_frames = [S_i.backup_frames image];
    S_i.backup_origins = [S_i.backup_origins Rt_WC];
else
    S_i.backup_origins = {S_i.backup_origins{2:end} Rt_WC};
    S_i.backup_frames = {S_i.backup_frames{2:end} image};
end

if length(S_i.X_i) < params.re_init_min_kps
 fprintf("\n Number of tracked points fell below %i, reinitialization...\n",...
     params.re_init_min_kps);
 S_i = re_init(S_i, K, params);
end
end