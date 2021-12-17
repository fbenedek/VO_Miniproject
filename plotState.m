function [t_WC_hist, n_landmark_hist] = plotState(fig, t_WC_hist, n_landmark_hist, img, state, T_WC)
figure(fig);
hist_length = 20;

assert(size(t_WC_hist,2)==size(n_landmark_hist,2),'History length mismatch')
t_WC = T_WC(1:3,4);

t_WC_hist = [t_WC_hist, t_WC];
n_landmark_hist = [n_landmark_hist, size(state.X,2)];

%plot current image, keypoints and keypoint candidates.
subplot(2,4,[1,2]);
imshow(img); hold on;
plot(state.P(1,:), state.P(2,:),'g+', 'MarkerSize', 6, 'LineWidth', 2);
plot(state.C(1,:), state.C(2,:),'r+', 'MarkerSize', 6, 'LineWidth', 2);
hold off;
title('Current image')
legend('Current keypoints', 'Candidate keypoints',  'Location','southeast');

% plot trajectoy and current landmarks
subplot(2,4,[3,4,7,8]);
plot(state.X(1,:),state.X(3,:),'g.')
hold on;
if size(t_WC_hist,2) < hist_length  
    plot(t_WC_hist(1,:), t_WC_hist(3,:), 'bo');
else
    plot(t_WC_hist(1,end-hist_length+1:end), t_WC_hist(3,end-hist_length+1:end), 'bo');
end
axis('equal');
title("Trajectory of last " + hist_length + " frames and current landmarks");
hold off;

subplot(2,4,5);
plot((-size(n_landmark_hist,2)+1):0, n_landmark_hist, 'bo');
title("# of tracked landmarks for last " + hist_length + " frames");
if size(n_landmark_hist,2) > 1
    xlim([-size(n_landmark_hist,2)+1,0])
end

subplot(2,4,6);
plot(t_WC_hist(1,:), t_WC_hist(3,:), 'bo');
axis('equal');
title("Full trajectory");
end

