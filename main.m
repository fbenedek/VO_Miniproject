  %% Setup
% read parameters from params.xml
params = readstruct("params.xml","FileType","xml");

ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    % need to set kitti_path to folder containing "05" and "poses"
    kitti_path = 'data/kitti/';
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/05.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many fi5374965328684e+03les of Malaga 7.
    malaga_path = 'data/malaga-urban-dataset-extract-07/';
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
    parking_path = 'data/parking/';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
elseif ds == 3
    % Path containing images and calibration data for GH dataset 1
    % Greenhouse dataset 1 - Machinery
    % Filtering and downsampling might be needed!
    greenhouse_1_path = 'data/greenhouse/01_machinery/';
    assert(exist('greenhouse_1_path', 'var') ~= 0);
    last_frame = 598;
    K = readxlsx([greenhouse_1_path '/cam_intrinsics.xlsx']);
elseif ds == 4
    % Path containing images and calibration data for GH dataset 1
    % Greenhouse dataset 2 - Long walk
    % Filtering and downsampling might be needed!
    greenhouse_2_path = 'data/greenhouse/02_long_walk/';
    assert(exist('greenhouse_2_path', 'var') ~= 0);
    last_frame = 4060;
    K = readxlsx([greenhouse_2_path '/cam_intrinsics.xlsx']);
elseif ds == 5
    % Path containing images and calibration data for GH dataset 1
    % Greenhouse dataset 3 - Short row
    % Filtering and downsampling might be needed!
    greenhouse_3_path = 'data/greenhouse/03_short_row/';
    assert(exist('greenhouse_3_path', 'var') ~= 0);
    last_frame = 598;
    K = readxlsx([greenhouse_3_path '/cam_intrinsics.xlsx']);
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [20,23];
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
elseif ds == 3
    img0 = rgb2gray(imread([greenhouse_1_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([greenhouse_1_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif ds == 4
    img0 = rgb2gray(imread([greenhouse_2_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([greenhouse_2_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
elseif ds == 5
    img0 = rgb2gray(imread([greenhouse_3_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([greenhouse_3_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

else
    assert(false);
end
addpath('functions/');
% init image size
params.image_size = size(img0);
% having img1, img2 we will run our triangulation here
[P_0, X_0, Twc_i] = bootstrap(img0, img1, K, params);

% TODO init state of the estimation:
[S_i, Twc_i] = init_state(P_0, X_0, Twc_i, img1, params);
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
% transformations from world frame to the camera frame of the first
% observation of each candidate keypoint.

fig = figure;
t_WC_hist = [];
n_landmark_hist = [];
[t_WC_hist, n_landmark_hist] = plotState(fig, t_WC_hist, n_landmark_hist, img1, S_i, Twc_i, params);
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
    elseif ds == 3
        image = im2uint8(rgb2gray(imread([greenhouse_1_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 4
        image = im2uint8(rgb2gray(imread([greenhouse_2_path ...
            sprintf('/images/img_%05d.png',i)])));
    elseif ds == 5
        image = im2uint8(rgb2gray(imread([greenhouse_3_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Markovian forward iteration
    [S_i, Twc_i] = process_frame(image, S_i, K, params);
    % Twc_i is 4*4 matrix containing the current pose
    
    [t_WC_hist, n_landmark_hist] = plotState(fig, t_WC_hist, n_landmark_hist, image, S_i, Twc_i, params);
    % Makes sure that plots refresh.    
    % Plot results
    % Plot the following:
    % Current image displaying tracked keypoints (P_i) and keypoint
    % candidates (C_i)
    % Number of tracked landmarks over last 20-ish frames
    % Plot of full estimated trajectory
    % Trajectory of the last 20 frames with 3D projection of landmarks
    pause(0.01);
    
    prev_img = image;
end