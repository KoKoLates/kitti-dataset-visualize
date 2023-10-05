#!/usr/bin/env python3

import cv2, os, rospy
import numpy as np
import sensor_msgs.point_cloud2 as pcl2

from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2


DATA_PATH = ('/home/aiseed/koko/catkin_ws/src/kitti_dataset_visualization'
             '/dataset/2011_09_26/2011_09_26_drive_0005_sync/')

if __name__ == '__main__':
    rospy.init_node('kitti_node', anonymous=True)
    frame: int = 0
    bridge: CvBridge = CvBridge()
    camera_publisher = rospy.Publisher('kitti_raw_image', Image, queue_size=10)
    pcloud_publisher = rospy.Publisher('kitti_point_cloud', PointCloud2, queue_size=10)

    while not rospy.is_shutdown():
        image = cv2.imread(os.path.join(DATA_PATH, f'image_02/data/{frame:010d}.png'))
        point_cloud = np.fromfile(
            os.path.join(DATA_PATH, f'velodyne_points/data/{frame:010d}.bin'), 
            dtype=np.float32).reshape(-1, 4)
        camera_publisher.publish(bridge.cv2_to_imgmsg(image, "bgr8"))

        header: Header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pcloud_publisher.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))

        rospy.loginfo('Topic published')
        rospy.Rate(10).sleep()
        frame = (frame + 1) % 154