#!/usr/bin/env python3

import rospy, cv2, os
import numpy as np
import sensor_msgs.point_cloud2 as pcl2

from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image, PointCloud2


def publish_camera(camera_publisher: rospy.Publisher, 
                   bridge: CvBridge, image: cv2.Mat) -> None:
    camera_publisher.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))

def publish_point_cloud(pcloud_publisher: rospy.Publisher, 
                        point_cloud: np.ndarray) -> None:
    header: Header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    pcloud_publisher.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))


def publish_ego_vehicle(ego_publisher: rospy.Publisher) -> None:
    marker: Marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()

    marker.id = 0
    marker.action = Marker.ADD
    marker.type = Marker.LINE_STRIP
    marker.lifetime = rospy.Duration()

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.scale.x = 0.1

    marker.points = []
    marker.points.append(Point(10,-10, 0))
    marker.points.append(Point( 0,  0, 0))
    marker.points.append(Point(10, 10, 0))
    ego_publisher.publish(marker)

