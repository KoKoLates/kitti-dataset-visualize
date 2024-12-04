# Kitti Dataset Visualization
Visualize bounding boxes and point cloud map on ROS. And also implements stereo visual odometry based on kitti dataset

## Dataset Visualization
Visualization and manipulation of raw data from multiple sensor of kitti dataset on ros, including video streaming, point cloud from lidar, GPS and IMU.

* Multi-sensor visualization
* Object detection and bounding box.

<div align="center">
  <img src="./assets/point_cloud_map.gif" alt="point cloud map" width="700">
</div>


## Stereo Visual Odometry
Using the stereo image captured from the kitti dataset to predict the depth and estimate the pose as odometry.

* Disparity map of stereo vision
* Depth estimation
* Visual odometry and accuracy metric.

<div align="center">
  <img src="./assets/disparity_map.png" alt="disparity" width="600">
  <img src="./assets/depth_map.png" alt="depth map" width="600">
</div>

## Reference
* [kitti dataset](https://www.cvlibs.net/datasets/kitti/)
