%% Setup
params = readstruct("params.xml","FileType","xml");

ds = 0; % 0: KITTI, 1: Malaga, 2: parking

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
else
    assert(false);
end

%% Bootstrap
% need to set bootstrap_frames
bootstrap_frames = [1,3];
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


addpath('functions/');
% patch_r = 4;
% [kpt, ldm] = init_VO(img0, img1, K, patch_r);
% visualizeLandmarks(img1, kpt, ldm);

% having img1, img2 we will run our triangulation here
[P_0, X_0, T_WC] = bootstrap(img0, img1, K, params);
% img0, img1 are the images returned by the bootstrap_frames indexes
% P_0 is a 2*K matrix containing the 2D positions of the keypoints on img0,
% this will be the frame of the origin.
% X_0 is a 3*K matrix containing the 3D positions of the keypoints

% TODO init state of the estimation:
[S, T_WC] = init_state(P_0, X_0, T_WC, img1, params);
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
[t_WC_hist, n_landmark_hist] = plotState(fig, t_WC_hist, n_landmark_hist, img1, S, T_WC, params);
% just for testing purposes
[t_WC_hist, n_landmark_hist] = plotState(fig, t_WC_hist, n_landmark_hist, img1, S, T_WC+2, params);

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
    % Makes sure that plots refresh.    
    pause(0.01);
    
    prev_img = image;
end