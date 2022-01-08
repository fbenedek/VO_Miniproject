function S_i = re_init(S_i, K, params)
% RE_INIT does a new bootstrap sequence if we run out of tracked points
[P_0, X_0, ~] = bootstrap(S_i.backup_frames{1}, S_i.backup_frames{end},...
    K, params);
Twc_i = S_i.backup_origins{end};
Twc_origin_tf = S_i.backup_origins{1};
Twc_origin_tf_homogenous = [Twc_origin_tf; 0 0 0 1]; 
S_i = init_state(P_0, X_0, Twc_i, S_i.backup_frames{end}, params);
S_i.X_i = (Twc_origin_tf_homogenous)*[S_i.X_i; ones(1, length(S_i.X_i))];
S_i.X_i = S_i.X_i(1:3,:);
end