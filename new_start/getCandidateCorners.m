function keypoints = getCandidateCorners(img, existing_kpts, params)
%GETCANDIDATECORNERS Summary of this function goes here
%   Detailed explanation goes here
corner_patch_size = params.corner_patch_size;
harris_kappa = params.harris_kappa;
num_kpts = params.num_new_candidates;
existing_r = params.existing_nonmaximum_supression_radius;
r = params.nonmaximum_supression_radius;
if params.use_shi_tomasi
    scores = shi_tomasi(img, corner_patch_size);
else
    scores = harris(img, corner_patch_size, harris_kappa);
end

% suppress around existing points.
temp_scores = padarray(scores, [existing_r existing_r]);
kpts_flipped = flipud(existing_kpts);
for i = 1:size(existing_kpts,2)
    kp = round(kpts_flipped(:, i)) + existing_r;
    temp_scores(kp(1)-existing_r:kp(1)+existing_r,...
        kp(2)-existing_r:kp(2)+existing_r) = zeros(2*existing_r + 1, 2*existing_r + 1);
end
scores = temp_scores(existing_r+1:end-existing_r,existing_r+1:end-existing_r);
keypoints = selectKeypoints(scores, num_kpts, r);

end

