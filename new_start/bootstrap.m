function [P_0, X_0, T_WC] = bootstrap(img0, img1, K, params)
% BOOTSTRAP return keypoints and landmarks for initialisation as well as
% camera frame transformation computed in the process.
%
% input:
%   img0: first image in the sequence
%   img2: image later in sequence to be used for bootstrap.
%
%Returns
%   P_0, a 2*M matrix of image coordinates on img1
%   X_0, a 3*M matrix of 3D positions, the triangulated pointcloud (the
%   camera frame from img0 is considered as the world frame
%   T_WC, transformation from world frame (img0) to camera frame from img1

corner_patch_size = 7;
harris_kappa = 0.1;
num_keypoints = 350;
nonmaximum_supression_radius = 9;
descriptor_radius = 9;
match_lambda = 4;
use_shi_tomasi = 0;

img0 = histeq(img0); img1 = histeq(img1);

if use_shi_tomasi
    score0 = shi_tomasi(img0, corner_patch_size);
    score1 = shi_tomasi(img1, corner_patch_size);
else
    score0 = harris(img0, corner_patch_size, harris_kappa);
    score1 = harris(img1, corner_patch_size, harris_kappa);
end
keypoints0 = selectKeypoints(score0, num_keypoints, nonmaximum_supression_radius);
keypoints1 = selectKeypoints(score1, num_keypoints, nonmaximum_supression_radius);

% Describe keypoints 
descriptors0 = describeKeypoints(img0, keypoints0, descriptor_radius);
descriptors1 = describeKeypoints(img1, keypoints1, descriptor_radius);

% match descriptors
matches = matchDescriptors(descriptors1, descriptors0, match_lambda);
[~, query_indices, match_indices] = find(matches);

kp1 = keypoints1(:, query_indices);
kp0 = keypoints0(:, match_indices);
figure(4)
imshow(img1)
hold on;
plot(kp1(2, :), kp1(1, :), 'rx', 'Linewidth', 2);
plot(kp0(2, :), kp0(1, :), 'bx', 'Linewidth', 2);

% convert to uv
kp0 = flipud(kp0); kp1 = flipud(kp1); 

[F,inliersIndex] = estimateFundamentalMatrix(kp0',kp1');
plot(kp1(1, inliersIndex), kp1(2, inliersIndex), 'ro', 'Linewidth', 2);
hold off;

kp0 = kp0(:, inliersIndex); kp1 = kp1(:, inliersIndex);
cameraParams = cameraIntrinsics([K(1), K(2,2)],[K(1,3), K(2,3)],params.image_size);
[relativeOrientation,relativeLocation, validRatio] = relativeCameraPose(F,cameraParams,kp0',kp1');
R_C1W = relativeOrientation';
t_C1W = -R_C1W*(relativeLocation');

%p1 = load('matches0001.txt');

M1 = K * eye(3,4);
M2 = K * [R_C1W, t_C1W];
X_0 = linearTriangulation([kp0;ones(1, size(kp0,2))],[kp1; ones(1, size(kp1,2))],M1,M2);
X_0_mask = X_0(3,:) > 0;
X_0 = X_0(:, X_0_mask);

P_0 = kp1(:, X_0_mask);
T_WC = [relativeOrientation, relativeLocation'];

projection_matrix = K * [R_C1W, t_C1W];
projected_points = projection_matrix * X_0;
projected_points = projected_points./projected_points (3,:);
reprojection_errors = projected_points(1:2,:) - P_0;
avg_repro_err = mean(vecnorm(reprojection_errors,1));

end
