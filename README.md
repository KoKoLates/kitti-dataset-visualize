# Kitti Dataset Visualization
Visualize the point cloud map, distances between objects, trajectories, and objects detected bounding boxes on ROS Rviz. And also implement stereo visual odometry with stereo disparity map and stereo depth estimation.

## Dataset Visualization
Visualization and manipulation of raw data from multiple sensors of the KITTI dataset on ROS RViz, including video streaming, point clouds from LiDAR, GPS, and IMU data.

- Multi-sensor visualization
- Object detection and bounding box.

<div align="center">
  <img src="./assets/pcl.png" alt="point_cloud_map" width="700">
  <img src="./assets/bbox.png" alt="bounding_boxes" width="700">
</div>

Please ensure that the dataset of point cloud map and images, and the calibration files are all proper in specific folder. And setup the `rosparam` in the [launch/main.launch](./launch/main.launch).

## Stereo Visual Odometry
Using the stereo image captured from the kitti dataset to predict the depth and estimate the pose as odometry.

* Disparity map of stereo vision
* Depth estimation
* Visual odometry and accuracy metric.

## Reference
* [kitti dataset](https://www.cvlibs.net/datasets/kitti/)
