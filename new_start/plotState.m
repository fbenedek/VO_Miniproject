function [t_WC_hist, n_landmark_hist] = plotState(fig, t_WC_hist, n_landmark_hist, img, state, T_WC, params)
% PLOTSTATE generates the plot as requested in project statement. 
% inputs:
%   fig: The figure containing the plot
%   t_WC_hist: A list containing the trajectory of translations. The
%   function adds the current translation based on T_WC and outputs the
%   updated trajectory for use in next iteration. At first call just pass
%   an empty list.
%   n_landmark_hist: List of the number of tracked landmarks through time.
%   The function outputs the updated list to use in next iteration. At
%   first iteration just pass an empty list.
%   img: the current image
%   state: the current state, S
%   T_WC: the current camera pose
%
% outputs:
%   t_WC_hist: Updated list containing the trajectory of translations.
%   n_landmark_hist: Updated List of the number of tracked landmarks 
%   through time.


figure(fig);
hist_length = params.plot_history_length;

assert(size(t_WC_hist,2)==size(n_landmark_hist,2),'History length mismatch')
t_WC = T_WC(1:3,4);

t_WC_hist = [t_WC_hist, t_WC];
n_landmark_hist = [n_landmark_hist, size(state.X_i,2)];

%plot current image, keypoints and keypoint candidates.
subplot(2,4,[1,2]);
imshow(img); hold on;
plot(state.P_i(1,:), state.P_i(2,:),'g+', 'MarkerSize', 6, 'LineWidth', 2);
plot(state.C_i(1,:), state.C_i(2,:),'r+', 'MarkerSize', 6, 'LineWidth', 2);
hold off;
title('Current image')
legend('Current keypoints', 'Candidate keypoints',  'Location','southeast');

% plot trajectoy and current landmarks
subplot(2,4,[3,4,7,8]);
plot(state.X_i(1,:),state.X_i(3,:),'g.')
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
if size(n_landmark_hist,2) < hist_length
    plot((-size(n_landmark_hist,2)+1):0, n_landmark_hist);
else
    plot((-hist_length+1):0, n_landmark_hist(end-hist_length+1:end));
end
title("# of tracked landmarks for last " + hist_length + " frames");
% 
% if size(n_landmark_hist,2) > 1
%     xlim([-size(n_landmark_hist,2)+1,0])
% end

subplot(2,4,6);
plot(t_WC_hist(1,:), t_WC_hist(3,:), 'bo');
axis('equal');
title("Full trajectory");
end

