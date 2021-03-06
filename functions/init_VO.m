function [keypoints, landmarks] = init_VO(img0, img1, K, patch_r)
%INIT_VO Initialises the visual odometry pipeline
%   keypoints:  the image coordinates of the matched features in img1.
%               NOTE: 2xM
%   ladmarks:   the 3D points triangulated from the matched features. The
%               world frame is assumed to be the camera frame of img0

% use matlab functions?
matlab_imp = 1;

num_kp = 100;
harris_kappa = 0.04;
nonmax_supression_radius = 16;
match_lambda = 20;


if matlab_imp > 0
    points0 = detectHarrisFeatures(img0, 'MinQuality', 0.01);
    points1 = detectHarrisFeatures(img1, 'MinQuality', 0.01);
    
%     tic;
%     D = pdist2(points1.Location, points0.Location, 'cityblock', 'Smallest', 1);
%     toc
    
    [features0,valid_points0] = extractFeatures(img0,points0);
    [features1,valid_points1] = extractFeatures(img1,points1);
    
    % Plots the found harris corners
    figure(1);
    subplot(1,2,1);
    imshow(img0); hold on;
    plot(valid_points0);
    hold off
    subplot(1,2,2);
    imshow(img1); hold on;
    plot(valid_points1);
    hold off
    
    indexPairs = matchFeatures(features0,features1,'MatchThreshold',15);
    
    matchedPoints0 = valid_points0(indexPairs(:,1),:);
    matchedPoints1 = valid_points1(indexPairs(:,2),:);
    
    % plot matched keypoints
    figure; 
    showMatchedFeatures(img0,img1,matchedPoints0,matchedPoints1);
    
    [F, inlier_idx] = estimateFundamentalMatrix(matchedPoints0,...
                        matchedPoints1,'Method','RANSAC','NumTrials', 2000);
    inlier_sum = sum(inlier_idx);
    E = K'*F*K;
    [Rots, u3] = decomposeEssentialMatrix(E);
    kp0 = matchedPoints0.Location(inlier_idx,:); kp0 = [kp0, ones(size(kp0,1),1)];
    kp1 = matchedPoints1.Location(inlier_idx,:); kp1 = [kp1, ones(size(kp1,1),1)];
    [R_CW, T_CW] = disambiguateRelativePose(Rots, u3, kp0', kp1', K, K);
    M1 = K * eye(3,4);
    M2 = K * [R_CW, T_CW];
    
    landmarks = linearTriangulation(kp0', kp1', M1,M2);
    keypoints = kp1(:,1:2)';
    
else
    harris0 = harris(img0, 2*patch_r+1, harris_kappa);
    harris1 = harris(img1, 2*patch_r+1, harris_kappa);
    
    keypoints0 = selectKeypoints(harris0, num_kp, nonmax_supression_radius);
    keypoints1 = selectKeypoints(harris1, num_kp, nonmax_supression_radius);
    
    % Plots the found harris corners
    figure(1);
    subplot(1,2,1);
    imshow(img0); hold on;
    plot(keypoints0(2,:), keypoints0(1,:),'o');
    hold off
    subplot(1,2,2);
    imshow(img1); hold on;
    plot(keypoints1(2,:), keypoints1(1,:),'o');
    hold off
    
    descriptors0 = describeKeypoints(img0, keypoints0, patch_r);
    descriptors1 = describeKeypoints(img1, keypoints1, patch_r);
    
    matches = matchDescriptors(descriptors1,descriptors0, match_lambda);
    indexPairs = matchFeatures(descriptors0',descriptors1','MatchThreshold',15);
    
    %NOTE, at this point we convert from [row;col] to [x,y]
    matched_keypoints0 = flipud(keypoints0(:,matches(matches>0)));
    matched_keypoints1 = flipud(keypoints1(:,matches>0));

%     matched_keypoints0 = flipud(keypoints0(:,indexPairs(:,1)));
%     matched_keypoints1 = flipud(keypoints1(:,indexPairs(:,2)));

    % plots the matched keypoints
    figure; 
    showMatchedFeatures(img0,img1,matched_keypoints0',matched_keypoints1');
    
    [F, inlier_idx] = estimateFundamentalMatrix(matched_keypoints0',...
                    matched_keypoints1','Method','RANSAC','NumTrials', 2000);
    
    matched_keypoints0 = matched_keypoints0(:,inlier_idx);
    matched_keypoints1 = matched_keypoints1(:,inlier_idx);
                       
    E = K'*F*K;
    [Rots, u3] = decomposeEssentialMatrix(E);
    
    % input has to be homogeneous coordinates. currently all we have is the
    % position in the image array.
    kp0 = [matched_keypoints0; ones(1,size(matched_keypoints0,2))];
    kp1 = [matched_keypoints1; ones(1,size(matched_keypoints1,2))];
    [R_CW, T_CW] = disambiguateRelativePose(Rots, u3, kp0, kp1, K, K);
    M1 = K * eye(3,4);
    M2 = K * [R_CW, T_CW];
    
    landmarks = linearTriangulation(kp0, kp1, M1,M2);
    keypoints = matched_keypoints1;
end

end