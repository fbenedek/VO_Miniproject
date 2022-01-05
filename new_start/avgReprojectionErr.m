function avg_repro_err = avgReprojectionErr(M,keypoints,landmarks)
projection_matrix = M;
if size(landmarks,1) < 4
    landmarks = [landmarks;ones(1,size(landmarks,2))];
end
projected_points = projection_matrix * landmarks;
projected_points = projected_points./projected_points (3,:);
reprojection_errors = projected_points(1:2,:) - keypoints;
avg_repro_err = mean(vecnorm(reprojection_errors,1));
end

