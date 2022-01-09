# VO-Miniproject
This repository contains the code for a MATLAB implementation of a Visual Odometry pipeline developed as a mini-project for the course [Vision Algorithms for Mobile Robots](http://rpg.ifi.uzh.ch/teaching.html) at UZH. 

The pipeline has been qualitatively evaluated on six different data sets, three of them are the benchmark datasets:
  * KITTI
  * Malaga
  * Parking Garage

The remaining are datasets collected by us, for testing the VO-pipeline in a significantly different setting than that of autonomous driving. The collected data comes from an industrial tomato greenhouse setting, which has been selected to emulate an environment for an agricultural robot.

## Replicating results
The parameters used have been slightly tuned to each data set, and have been saved in seperate XML-files that are loaded from the main file alongside the relevant images. To switch between datasets simply change the `ds` parameter at the top of main:
  - 0: KITTI [video link](https://youtu.be/4XeiFHxf4Sk)
  - 1: Malaga [video link](https://youtu.be/ptTxC_Pkka8)
  - 2: Parking garage [video link](https://youtu.be/XaaNMHqJkvc)
  - 3-5: Greenhouses [video link](https://youtu.be/KdPg5T4fGCE), [video link](https://youtu.be/ZgZsuuS9pY4), [video link](https://youtu.be/dm3dDU9VfY8)

The pipeline has been executed on the following machines:
- Intel Core i7-1165G7 @ 2.80GHz, 8Gb RAM (MATLAB 2021b, 3650 threads)
- Intel Core i7-7700HQ @ 2.80GHz, 8Gb RAM (MATLAB R2021a, 4400 threads)
- Intel Core i7-1165G7 @ 2.80GHz, 16Gb RAM (MATLAB R2021a)


## Tuning the parameters
Each parameter file has the following noteworthy entries:
- `bootstrap_repro_error`
    - Reprojection error threshold for recomputing the initial landmarks. If the execution gets stuck in initialisation, increase this.
- `points_num_target`
    - Used for determining whether to add new candidates. If the sum of current keypoints and candidates fall below this, new ones are added.
- `point/candidate_tracker_###`
    - Parameters of the vision.PointTracker objects used for tracking keypoints and candidates.
- `ransac_max_error`
    - The error tolerated for inliers when estimating camera pose using RANSAC
- `max_rp_err_for_triangulation`
    - A threshold for the reprojection error allowed when triangulating new landmarks. If a triangulated landmarks has higher reprojection error it is discarded. If you do not get enough new landmarks try increasing this, but be aware that too poor triangulation will lead to instability.
- `use_simple_triangulation_criteria`
    - By default new landmarks are triangulated based on the bearing angle in frame from first observation to the current, but as a sanitiy check this triangulation criteria uses only the pixel distance between the candidates in the two frames.
- `reproject_landmarks`
    - If 1, reprojects the landmarks to the current frame after each iteration. Thought to help maintain landmarks.
- `depth_thresh`
    - Newly triangulated landmarks are filtered based on their distance to the camera relative to the median of the other points. The higher this threshold, the farther away points are allowed to be triangulated. Honestly, this heuristic has not had much impact and is mostly relevant for initialisation.
- `candidate_dist_threshold`
    - To ensure taht new candidates are not found directly on top of existing keypoints, this sets a radius around existing keypoints and candidates where new ones are not proposed.
- `min_bearing_angle`
    - The bearing angle used for the threshold to determine when to triangulate candidates to landmarks
- `ba_max_length`
    - Width of the bundle adjustment window
- `re_init_min_kps`
    - The threshold for when to completely re-initialise landmarks. Sometimes the performance seems to degrade leading to increasingly fewer maintained landmarks. When below this threshold, landmarks are completely re-initialised, i.e. bootstrapping sort of starts over.
