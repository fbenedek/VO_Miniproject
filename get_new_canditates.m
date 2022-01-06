function S_i = get_new_canditates(image, S_i, Rt_WC, params)
% GET_NEW_CANDIDATES returns a new state that contains a C_i to which new
% candidate points are added

% get fields from S_i to avoid visual clutter

P_i = S_i.P_i;
X_i = S_i.X_i;
C_i = S_i.C_i;
F_i = S_i.F_i;
Tau_i = S_i.Tau_i;

if size(S_i.P_i,2) + size(S_i.C_i,2) < params.points_num_target
    C_new = double(getCornersSpread(image, [S_i.P_i, S_i.C_i],params.num_continous_candidates, params).Location');
else
    C_new = [];
end

curr_Tau = Rt_WC(:);
%1*12 vectorized current Twc_i

Tau_new = repmat(curr_Tau, 1, size(C_new,2));
%repmat of curr_Tau (as all new points have the same Tau)

% output the refreshed points
S_i.P_i = P_i;
S_i.X_i = X_i;
S_i.C_i = [C_i, C_new];
S_i.F_i = [F_i, C_new];
S_i.Tau_i = [Tau_i, Tau_new];

end