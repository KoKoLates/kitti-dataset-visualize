# Kitti Dataset Visualization

This repository visualizes the dataset using a point cloud map, the distances between detected objects, the trajectories of moving objects, and annotated bounding boxes in RViz with ROS noetic. It also implements visual odometry using stereo depth estimation.


## Dataset Visualization

This section processes raw data from multiple sensors, including a camera, IMU, GPS, and LiDAR, from the dataset. It creates collectors and publishers in ROS to publish the processed data using visualization tools. The published topics include:

- Multiple sensors
- Annotated detection bounding boxes
- Distances between objects and their trajectories


<div align="center">
  <img src="./assets/pcl.png" alt="point_cloud_map" width="700">
  <img src="./assets/bbox.png" alt="bounding_boxes" width="700">
</div>

Please ensure that all required datasets are prepared and that the paths to the corresponding files and folders are correctly set in the `rosparam` file located at [launch/main.launch](./launch/main.launch). Alternatively, you can set the default path directly in the main function using:

```python
rospy.get_param('param_name', 'default path of file of folder')
```

There are three paths you need to be aware of: 

1. `calib_path`: Stores the file for camera and Velodyne calibrations.
2. `track_path`: The file containing annotated objects tracked in each frame.
3. `value_path`: Stores the path to the folder containing all sensor data, such as images, Velodyne points, and OXTS data.

Once the parameters and dataset are properly set up, you can compile the package using `catkin_make` and launch it with `roslaunch`. This will start the ROS master, the necessary nodes, and RViz in the same terminal.

```shell
roslaunch kitt__dataset_visualize launch/main.launch
```

Before running the command, ensure that your shell can recognize the packages by sourcing the workspace setup file:

```
source devel/setup.bash
```

To avoid having to source this manually every time, you can add the following line to your `.bashrc` file

## Stereo Visual Odometry
Using the stereo image captured from the kitti dataset to predict the depth and estimate the pose as odometry.

* Disparity map of stereo vision
* Depth estimation
* Visual odometry and accuracy metric.

<div align="center">
  <img src="./assets/stereo_bm.png" alt="stereo bm matcher" width="600">
  <img src="./assets/stereo_sgbm.png" alt="stereo sgbm matcher" width="600">
</div>

Matching the disparity map by stereo vision with `BM` and `SGBM` matcher respectively. And based on the disparity map, derived the depth map with intrinsic parameter of left camera.

<div align="center">
  <img src="./assets/depth_map.png" alt="depth map" width="600">
</div>

`cv2.solvePnPRansac` function solves this problem by minimizing the reprojection error, which measures how well the projected 3D points align with the observed 2D keypoints
<div align="center">
  <img src="./assets/monocular.png" alt="monocular" width="350" />
  <img src="./assets/stereo.png" alt="stereo" width="350" />
</div>
