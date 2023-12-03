#!/usr/bin/env python3

import os

from collectors import *
from publishers import *
from sensor_msgs.msg import Image, PointCloud2

DATA_PATH = ('/home/aiseed/koko/catkin_ws/src/kitti_dataset_visualization'
             '/dataset/2011_09_26/2011_09_26_drive_0005_sync/')

if __name__ == '__main__':
    rospy.init_node('kitti_node', anonymous=True)
    frame: int = 0
    bridge: CvBridge = CvBridge()
    
    cam_pub = rospy.Publisher('kitti_img', Image,       queue_size=10)
    pcl_pub = rospy.Publisher('kitti_pcl', PointCloud2, queue_size=10)
    imu_pub = rospy.Publisher('kitti_imu', Imu,         queue_size=10)
    gps_pub = rospy.Publisher('kitti_gps', NavSatFix,   queue_size=10)
    ego_pub = rospy.Publisher('kitti_ego', MarkerArray, queue_size=10)

    while not rospy.is_shutdown():
        imu = read_imu(os.path.join(DATA_PATH, f'oxts/data/{frame:010d}.txt')) 
        img = read_img(os.path.join(DATA_PATH, f'image_02/data/{frame:010d}.png'))
        pcl = read_pcl(os.path.join(DATA_PATH, f'velodyne_points/data/{frame:010d}.bin'))

        publish_img(cam_pub, bridge, img)
        publish_pcl(pcl_pub, pcl)
        publish_ego(ego_pub)
        publish_imu(imu_pub, imu)
        publish_gps(gps_pub, imu)

        rospy.Rate(10).sleep()
        frame = (frame + 1) % 154
