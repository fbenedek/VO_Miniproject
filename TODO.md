# TODO
A brief description of the tasks we will have to complete. Trying to make a bit more sense of the descriptions from the project description. What follows is thought to be a general overview of the tasks needed to be completed, including subtasks/problems, as well as some potentially interesting avenues for investigation. 

## Tasks
Tasks in *italics* can be considered optional
  - [ ] Implementation of initialization with manual frame selection.
  - [ ] *Investigation of automatic initialization*
  - [ ] Match keypoints between two frames using KLT (see MATLAB vision.PointTracker) and reassociate ladmarks based on this.
  - [ ] Experiment with `ransacLocalization` to find fitting inlier threshold.
  - [ ] Find new candidate keypoints *and ensure that these are distinct*
  - [ ] Track candidate keypoints and triangulate when appropriate.
  - [ ] Combine everything to fit the framework described in the project statement
  - [ ] Visualising the results using the subplots.
  - [ ] Collect custom dataset
  - [ ] Calibrate camera for custom data set
  - [ ] Run pipeline on custom dataset

  - [ ] *Compare KLT to matching Harris features on the custom dataset. (Assumption, KLT may be significantly better due to repeated patterns)*



## Notation
Some useful definitions
  - **Keypoints**: Points of interest in an image, e.g. Harris corners. keypoints always refer to 2D-points.
  - **Landmarks**: 3D points in world coordinates. These should have associated keypoints. 
## Initialization
Initialization will be run once to begin the VO pipeline, It includes the following steps:
1. Choose appropriate frames
    - Can be selected manually through experimentation
    - We could develop a method for selecting this automatically. Maybe in similar fashion to what is presented in the section about triangulation of new keypoints.
    - It would also be interesting to comopare the results to the rule of thumb presented in lecture 10.
2. Find keypoint matches in the two images. E.g. find keypoints in both images and match their descriptors. Different approaches to this exists.
    - The TAs strongly reccomend to use KLT.
    - We could also use Harris features and match them similar to exercise 3.
    - It might be interesting to invesigate using SIFT.
3. Estimate the pose (R, T) between the two frames
    - E.g Using the 8-point algorithm.
4. Using the pose and the camera calibration matrix, K, we can triangulate 3D poitns using least-squares (see exercise 6, linearTriangulation.m)
5. Repeat 3-4 using a RANSAC approach to find the best (R, T) and associated landmarks
6. Return the inlier landmarks and associated keypoints (from frame 2 I assume)

**Important note:** We can use the MATLAB function `estimateFundamentalMatrix`, which already implements RANSAC. It will return the fundamental matrix, F, and the the inlier indices. This essentially takes care of steps 3-5. Through this approach, however, we cannot make use of knowledge of the camera calibration matrix, K, in the RANSAC step. I am unsure whether this makes a difference, but it might be worth investigating.

## Continous Operation
The continous operation is divided into two parts, associating current keypoints to existing landmarks to get the current camera pose, and asynchronously triangulating new landmarks. 

### Associating current keypoints to existing landmarks to get the current camera pose
1. Get get keypoints . 
    - We can do this using Harris descriptors and `matchDescriptors` from exercise 3 (this matches the keypoint descriptors from frame (i-1) to i, from which we can determine the landmark associations, as the current landmarks are associated with the keypoints of frame (i-1)), **but it is recommended to use KLT**.
2. Use `ransacLocalization` from exercise 7 with the matched keypoints and landmarks to get the current pose and inlier mask to mask out outlier keypoints and landmarks. We might want to experiment with an inlier threshold. We also migth want to deactivate the DLT part of `ransacLocalization`. 
3. Update the state pased on the new pose and new inliers.

### Triangulating new landmarks
This part is somewhat difficult to understand based on the project description, so take this part with a grain of salt.

At each frame do the following
  - Track previous candidate keypoints. I.e. get the new keypoint coordinates of keypoints from the previous frame. What is saved in the state is only the current keypoint coordinates and the original keypoint coordinates from when the tracking began (along with the camera pose at that point).
    - If the angle between the bearing vectors of a keypoint pair (i.e. the point at start of tracking and the current) is larger than a certain threshold, we can triangulate that point to a landmark, thus adding it to the list of landmarks along with the associated current keypoint. The project statement refers to exercise 5 for triangulation, but this seems to be false.
  - Find new keypoint candidates. How to do this I am not quite sure, but I assume it is just a matter of finding Harris features. We also probably should make sure that these new candidate keypoints are distinct from the existing candidate keypoints and the keypoints assocaited to existing landmarks. How to do this I am also not sure.
