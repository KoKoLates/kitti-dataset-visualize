# Kitti Dataset Visualization
Visualize the point cloud map, distance, trajectory, and object bounding boxes with ROS Rviz. And also implements stereo visual odometry with stereo disparity map and stereo depth estimation.

## Dataset Visualization
Visualization and manipulation of raw data from multiple sensors of the KITTI dataset on ROS RViz, including video streaming, point clouds from LiDAR, GPS, and IMU data.
* Multi-sensor visualization
* Object detection and bounding box.

<div align="center">
  <img src="./assets/point-cloud-map.PNG" alt="point cloud map" width="700">
  <img src="./assets/object-detecting.PNG" alt="camera" width="700">
</div>


## Stereo Visual Odometry
Using the stereo image captured from the kitti dataset to predict the depth and estimate the pose as odometry.

* Disparity map of stereo vision
* Depth estimation
* Visual odometry and accuracy metric.

<div align="center">
  <img src="./assets/disparity-map.png" alt="disparity" width="600">
  <img src="./assets/depth-map.png" alt="depth map" width="600">
</div>

## Reference
* [kitti dataset](https://www.cvlibs.net/datasets/kitti/)
