#!/usr/bin/env python3

import cv2
import rospy
import numpy as np
import pandas as pd
import sensor_msgs.point_cloud2 as pcl2

from cv_bridge import CvBridge
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Imu, NavSatFix
from tf.transformations import quaternion_from_euler


def publish_img(img_pub: rospy.Publisher, 
                bridge: CvBridge, image: cv2.Mat) -> None:
    """
    @param img_pub: ros publisher of camera image.
    @param bridge : cv bridge for image format conversion.
    @param image  : processing image frame.
    """
    img_pub.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))


def publish_pcl(pcl_pub: rospy.Publisher, point_cloud: np.ndarray) -> None:
    """
    @param pcl_pub: ros publisher of point cloud map.
    @param point_cloud: the raw data of point clouds.
    """
    header: Header  = Header()
    header.frame_id = 'map'
    header.stamp    = rospy.Time.now()
    pcl_pub.publish(pcl2.create_cloud_xyz32(header, point_cloud[:, :3]))


def publish_ego(ego_pub: rospy.Publisher) -> None:
    """
    @param ego_pub: 
    """
    marker_array: MarkerArray = MarkerArray()

    ## marker for view point line 
    marker: Marker          = Marker()
    marker.header.frame_id  = 'map'
    marker.header.stamp     = rospy.Time.now()

    marker.id       = 0
    marker.type     = Marker.LINE_STRIP
    marker.action   = Marker.ADD
    marker.lifetime = rospy.Duration()

    marker.color.r: float = 0.0
    marker.color.g: float = 1.0
    marker.color.b: float = 0.0
    marker.color.a: float = 1.0
    marker.scale.x: float = 0.1

    marker.points = []
    marker.points.append(Point(10,-10, 0))
    marker.points.append(Point( 0,  0, 0))
    marker.points.append(Point(10, 10, 0))

    ## mesh marker for vehicle models
    mesh_marker: Marker         = Marker()
    mesh_marker.header.frame_id = 'map'
    mesh_marker.header.stamp    = rospy.Time.now()

    mesh_marker.id            = -1
    mesh_marker.type          = Marker.MESH_RESOURCE
    mesh_marker.lifetime      = rospy.Duration()
    mesh_marker.mesh_resource = 'package://kitti_dataset_visualization/rviz/vehicle.dae'

    mesh_marker.pose.position.x: float =  0.0
    mesh_marker.pose.position.y: float =  0.0
    mesh_marker.pose.position.z: float = -1.73

    quater = quaternion_from_euler(np.pi / 2, 0, np.pi)
    mesh_marker.pose.orientation.x = quater[0]
    mesh_marker.pose.orientation.y = quater[1]
    mesh_marker.pose.orientation.z = quater[2]
    mesh_marker.pose.orientation.w = quater[3]

    mesh_marker.color.r: float = 1.0
    mesh_marker.color.g: float = 1.0
    mesh_marker.color.b: float = 1.0
    mesh_marker.color.a: float = 1.0
    mesh_marker.scale.x: float = 0.9
    mesh_marker.scale.y: float = 0.9
    mesh_marker.scale.z: float = 0.9

    marker_array.markers.append(marker)
    marker_array.markers.append(mesh_marker)

    ego_pub.publish(marker_array)


def publish_imu(imu_pub: rospy.Publisher, data: pd.DataFrame) -> None:
    imu: Imu = Imu()
    imu.header.frame_id = 'map'
    imu.header.stamp = rospy.Time.now()

    quater = quaternion_from_euler(float(data.roll ), 
                                   float(data.pitch),
                                   float(data.yaw  ))
    imu.orientation.x = quater[0]
    imu.orientation.y = quater[1]
    imu.orientation.z = quater[2]
    imu.orientation.w = quater[3]

    imu.linear_acceleration.x = data.af 
    imu.linear_acceleration.y = data.al 
    imu.linear_acceleration.z = data.au

    imu.angular_velocity.x = data.wf 
    imu.angular_velocity.y = data.wl 
    imu.angular_velocity.z = data.wu

    imu_pub.publish(imu)


def publish_gps(gps_pub: rospy.Publisher, data: pd.DataFrame) -> None:
    """"""
    gps: NavSatFix = NavSatFix()
    gps.header.frame_id = 'map'
    gps.header.stamp = rospy.Time.now()

    gps.latitude    = data.lat 
    gps.altitude    = data.alt
    gps.longitude   = data.lon

    gps_pub.publish(gps)

