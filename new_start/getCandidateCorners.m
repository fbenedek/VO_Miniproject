function keypoints = getCandidateCorners(img, existing_kpts, params)
%GETCANDIDATECORNERS Summary of this function goes here
%   Detailed explanation goes here
corner_patch_size = params.corner_patch_size;
harris_kappa = params.harris_kappa;
num_kpts = params.num_new_candidates;
r = params.nonmaximum_supression_radius;
if params.use_shi_tomasi
    scores = shi_tomasi(img, corner_patch_size);
else
    scores = harris(img, corner_patch_size, harris_kappa);
end

% suppress around existing points.
temp_scores = padarray(scores, [r r]);
kpts_flipped = flipud(existing_kpts);
for i = 1:size(existing_kpts,2)
    kp = round(kpts_flipped(:, i)) + r;
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = zeros(2*r + 1, 2*r + 1);
end
scores = temp_scores(r+1:end-r,r+1:end-r);
keypoints = selectKeypoints(scores, num_kpts, r);

end

