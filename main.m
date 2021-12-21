%% Setup
% read parameters from params.xml
params = readstruct("params.xml","FileType","xml");
ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
if ds == 0
    img0 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/05/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end
% having img1, img2 we will run our triangulation here
% TODO: P_0, X_0 = bootstrap(img0, img1), where:
% img0, img1 are the images returned by the bootstrap_frames indexes
% P_0 is a 2*K matrix containing the 2D positions of the keypoints on img0,
% this will be the frame of the origin.
% X_0 is a 3*K matrix containing the 3D positions of the keypoints

% TODO init state of the estimation:
% S_i, Twc_i = init_state(P_0, X_0, img0, img1), where:
% P_0 is a 2*K matrix containing the 2D positions of the keypoints on img0
% X_0 is a 3*K matrix containing the 3D positions of the keypoints
% Twc_i is 4*4 matrix containing the current pose, now the pose of the
% frame of img1 (can also be vectorized)
% S_i is a struct, with the following fields:
% P_i is a 2*K matrix containing the 2D positions of the keypoints on img1
% X_i is a 3*K matrix containing the 3D positions of the keypoints
% C_i is a 2*M matrix containing keypoints candidates from img1, which are
% not similar to P_i but are Harris corners
% F_i is a 2*M matrix and stores the first observations of new keypoint
% candidates. Now it is identical to C_i.
% Tau_i is a 12*M matrix containing the concatenated and vectorized
% transformations from the first observation of each keypoint candidate to
% the current frame. Now contains M identity transformations only.

%% Continuous operation
range = (bootstrap_frames(2)+1):last_frame;
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/05/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % TODO implement Markovian forward iteration
    % [S_i, Twc_i] = process_frame(image, S_i, params), where:
    % Twc_i is 4*4 matrix containing the current pose
    % (can also be vectorized)
    % S_i is a struct with the following fields:
    % P_i is a 2*K matrix containing the 2D positions of the keypoints on
    % prev_img
    % X_i is a 3*K matrix containing the 3D positions of the keypoints
    % C_i is a 2*M matrix containing keypoints candidates from prev_img, which are
    % not similar to P_i but are Harris corners
    % F_i is a 2*M matrix and stores the first observations of new keypoint
    % candidates.
    % Tau_i is a 12*M matrix containing the concatenated and vectorized
    % transformations from the first observation of each keypoint candidate to
    % the current frame.
    
    % D_i = plot_state(S_i, image)

    % Makes sure that plots refresh.    
    % TODO: Plot results
    % Plot the following:
    % Current image displaying tracked keypoints (P_i) and keypoint
    % candidates (C_i)
    % Number of tracked landmarks over last 20-ish frames
    % Plot of full estimated trajectory
    % Trajectory of the last 20 frames with 3D projection of landmarks
    pause(0.01);
    
    prev_img = image;
end