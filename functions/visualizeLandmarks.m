function visualizeLandmarks(img, keypoints,landmarks)
%VISUALIZELANDMARKS Ploits the 3D landmarks
sz = landmarks(3,:); sz = normalize(sz,'range');
figure; 
subplot(1,2,1);
imagesc(img); hold on;
scatter(keypoints(1,:)', keypoints(2,:)', sz*50+1, 'filled');
hold off
subplot(1,2,2);
plot3(landmarks(1,:), landmarks(2,:), landmarks(3,:), 'o');
pbaspect([1 1 1])
end

