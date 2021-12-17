function c = getNewCandidateKeypoints(img,P_i)
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

% The threshold for being too close to existing points
lambda = 10;

% find harris corners
points = detectHarrisFeatures(img, 'MinQuality', 0.1);

%tic;
points = nonMaxSupression(points, 10);
%toc

% find the smallest distance to existing points
D = pdist2(P_i', points.Location, 'cityblock', 'Smallest', 1);
distinct_points = D > lambda;

% figure;
% imshow(img); hold on;
% plot(points(distinct_points,:));

c = points.Location(distinct_points,:)';
end

