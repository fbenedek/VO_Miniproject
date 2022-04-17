function c = getNewCandidateKeypoints(img,P_i, params)
%getCandidateKeypoints generates candidate HArris corners in the current
%image, returns on those distinct from existing points
%
%   input:
%   img: the current image
%   P_i: existing keypoints 2xM (should be a concatenation of P_i and C_i)
%
%   output:
%   c: the candidate keypoints, 2xM
%
%   NOTE: Currently we simply distinguish keypoints based on the manhattan
%   distance in the image. Other heuristics may be better.

min_harris_quality = params.min_harris_feature_quality;
non_max_suppression_radius = params.non_max_suppression_radius;
dist_type = params.proposal_test_norm;

% The threshold for being too close to existing points
lambda = params.candidate_dist_threshold;

% find harris corners
points = detectHarrisFeatures(histeq(img), 'MinQuality', min_harris_quality);
points = detectFASTFeatures(img)

%tic;
points = nonMaxSupression(points, non_max_suppression_radius);
%toc

% find the smallest distance to existing points
D = pdist2(P_i', points.Location, dist_type, 'Smallest', 1);
distinct_points = D > lambda;

% figure;
% imshow(img); hold on;
% plot(points(distinct_points,:));

c = points.Location(distinct_points,:)';
end

